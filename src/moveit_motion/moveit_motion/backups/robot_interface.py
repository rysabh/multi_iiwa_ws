import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState

from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
)
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK
from moveit_msgs.action import ExecuteTrajectory

from ros_submodules.wait_for_message import wait_for_message


class RobotInterfaceNode(Node):
    timeout_sec_ = 5.0

    move_group_name_ = "arm"
    namespace_ = "lbr"

    joint_state_topic_ = "joint_states"
    plan_srv_name_ = "plan_kinematic_path"
    ik_srv_name_ = "compute_ik"
    fk_srv_name_ = "compute_fk"
    execute_action_name_ = "execute_trajectory"

    base_ = "link_0"
    end_effector_ = "link_ee"

    def __init__(self) -> None:
        super().__init__("robot_interface_node", namespace=self.namespace_)

        self.ik_client_ = self.create_client(GetPositionIK, self.ik_srv_name_)
        if not self.ik_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error("IK service not available.")
            exit(1)

        self.fk_client_ = self.create_client(GetPositionFK, self.fk_srv_name_)
        if not self.fk_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error("FK service not available.")
            exit(1)

        self.plan_client_ = self.create_client(GetMotionPlan, self.plan_srv_name_)
        if not self.plan_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error("Plan service not available.")
            exit(1)

        self.execute_client_ = ActionClient(
            self, ExecuteTrajectory, self.execute_action_name_
        )
        if not self.execute_client_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error("Execute action not available.")
            exit(1)

    def get_ik(self, target_pose: Pose) -> JointState | None:
        request = GetPositionIK.Request()

        request.ik_request.group_name = self.move_group_name_
        tf_prefix = self.get_namespace()[1:]
        request.ik_request.pose_stamped.header.frame_id = f"{tf_prefix}/{self.base_}"
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.avoid_collisions = True

        future = self.ik_client_.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Failed to get IK solution")
            return None

        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        return response.solution.joint_state
    
    def get_fk(self) -> Pose | None:
        current_joint_state_set, current_joint_state = wait_for_message(
            JointState, self, self.joint_state_topic_, time_to_wait=1.0
        )
        if not current_joint_state_set:
            self.get_logger().error("Failed to get current joint state")
            return None

        current_robot_state = RobotState()
        current_robot_state.joint_state = current_joint_state

        request = GetPositionFK.Request()

        request.header.frame_id = f"{self.namespace_}/{self.base_}"
        request.header.stamp = self.get_clock().now().to_msg()

        request.fk_link_names.append(self.end_effector_)
        request.robot_state = current_robot_state

        future = self.fk_client_.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Failed to get FK solution")
            return None
        
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Failed to get FK solution: {response.error_code.val}"
            )
            return None
        
        return response.pose_stamped[0].pose

    def sum_of_square_diff(
        self, joint_state_1: JointState, joint_state_2: JointState
    ) -> float:
        return np.sum(
            np.square(np.subtract(joint_state_1.position, joint_state_2.position))
        )

    def check_same_pose(
            self, pose_1: Pose, pose_2: Pose
    ) -> bool:
        x_diff = np.abs(pose_1.position.x - pose_2.position.x)
        y_diff = np.abs(pose_1.position.y - pose_2.position.y)
        z_diff = np.abs(pose_1.position.z - pose_2.position.z)

        pos_diff = np.asarray([x_diff, y_diff, z_diff])

        qx_diff = np.abs(pose_1.orientation.x - pose_2.orientation.x)
        qy_diff = np.abs(pose_1.orientation.y - pose_2.orientation.y)
        qz_diff = np.abs(pose_1.orientation.z - pose_2.orientation.z)
        qw_diff = np.abs(pose_1.orientation.w - pose_2.orientation.w)

        quat_diff = np.asarray([qx_diff, qy_diff, qz_diff, qw_diff])

        return np.sum(np.square(pos_diff)) < 0.001 and np.sum(np.square(quat_diff)) < 0.001

    def get_best_ik(self, target_pose: Pose, attempts: int = 100) -> JointState | None:
        current_joint_state_set, current_joint_state = wait_for_message(
            JointState, self, self.joint_state_topic_, time_to_wait=1.0
        )
        if not current_joint_state_set:
            self.get_logger().error("Failed to get current joint state")
            return None

        best_cost = np.inf
        best_joint_state = None

        for _ in range(attempts):
            joint_state = self.get_ik(target_pose)
            if joint_state is None:
                continue

            cost = self.sum_of_square_diff(current_joint_state, joint_state)
            if cost < best_cost:
                best_cost = cost
                best_joint_state = joint_state

        if not best_joint_state:
            self.get_logger().error("Failed to get IK solution")

        return best_joint_state

    def get_motion_plan(
        self, target_pose: Pose, linear: bool = False, attempts: int = 10
    ) -> RobotTrajectory | None:
        current_pose = self.get_fk()
        if not current_pose:
            self.get_logger().error("Failed to get current pose")

        if self.check_same_pose(current_pose, target_pose):
            return RobotTrajectory()

        current_joint_state_set, current_joint_state = wait_for_message(
            JointState, self, self.joint_state_topic_, time_to_wait=1.0
        )
        if not current_joint_state_set:
            self.get_logger().error("Failed to get current joint state")
            return None

        current_robot_state = RobotState()
        current_robot_state.joint_state.position = current_joint_state.position

        target_joint_state = self.get_best_ik(target_pose)
        if target_joint_state is None:
            self.get_logger().error("Failed to get target joint state")
            return None

        target_constraint = Constraints()
        for i in range(len(target_joint_state.position)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = target_joint_state.name[i]
            joint_constraint.position = target_joint_state.position[i]
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            target_constraint.joint_constraints.append(joint_constraint)

        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.move_group_name_
        request.motion_plan_request.start_state = current_robot_state
        request.motion_plan_request.goal_constraints.append(target_constraint)
        request.motion_plan_request.num_planning_attempts = 10
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.1
        request.motion_plan_request.max_acceleration_scaling_factor = 0.1

        if linear:
            request.motion_plan_request.pipeline_id = "pilz_industrial_motion_planner"
            request.motion_plan_request.planner_id = "LIN"
        else:
            request.motion_plan_request.pipeline_id = "ompl"
            request.motion_plan_request.planner_id = "APSConfigDefault"

        for _ in range(attempts):
            plan_future = self.plan_client_.call_async(request)
            rclpy.spin_until_future_complete(self, plan_future)

            if plan_future.result() is None:
                self.get_logger().error("Failed to get motion plan")

            response = plan_future.result()
            if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(
                    f"Failed to get motion plan: {response.motion_plan_response.error_code.val}"
                )
            else:
                return response.motion_plan_response.trajectory
            
        return None

    def get_motion_execute_client(self) -> ActionClient:
        return self.execute_client_


def main(args=None):
    rclpy.init(args=args)

    robot_interface_node = RobotInterfaceNode()
    robot_interface_node.get_logger().info("Robot interface node started.")

    target_poses = []
    for i in range(3):
        target_poses.append(
            Pose(
                position=Point(x=0.6, y=-0.1 + 0.1 * i, z=0.6),
                orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
            )
        )

    traj = robot_interface_node.get_motion_plan(target_poses[1])
    if traj:
        client = robot_interface_node.get_motion_execute_client()
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(robot_interface_node, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            robot_interface_node.get_logger().error("Failed to execute trajectory")
        else:
            robot_interface_node.get_logger().info("Trajectory accepted")

        result_future = goal_handle.get_result_async()

        expect_duration = traj.joint_trajectory.points[-1].time_from_start
        expect_time = time.time() + 2 * expect_duration.sec 
        while not result_future.done() and time.time() < expect_time:
            time.sleep(0.01)

        robot_interface_node.get_logger().info("Trajectory executed")

        robot_interface_node.get_logger().info("Current pose: " + str(robot_interface_node.get_fk())) 

    for target_pose in target_poses:
        traj = robot_interface_node.get_motion_plan(target_pose, True)
        if traj:
            client = robot_interface_node.get_motion_execute_client()
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = traj

            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(robot_interface_node, future)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                robot_interface_node.get_logger().error("Failed to execute trajectory")
            else:
                robot_interface_node.get_logger().info("Trajectory accepted")

            
            result_future = goal_handle.get_result_async()

            expect_duration = traj.joint_trajectory.points[-1].time_from_start
            expect_time = time.time() + 2 * expect_duration.sec
            while not result_future.done() and time.time() < expect_time:
                time.sleep(0.01)

            robot_interface_node.get_logger().info("Trajectory executed")
        
            robot_interface_node.get_logger().info("Current pose: "  + str(robot_interface_node.get_fk()) )

    rclpy.spin(robot_interface_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()