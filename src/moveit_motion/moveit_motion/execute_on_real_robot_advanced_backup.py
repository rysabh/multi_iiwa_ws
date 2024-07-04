import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from submodules.wait_for_message import wait_for_message
import pickle
import numpy as np
import csv

def save_trajectory(trajectory_data, filename):
    dir_path = "/home/cam/multi_iiwa_ws/src/moveit_motion/moveit_motion/robot_trajectories"
    full_path = os.path.join(dir_path, filename)
    with open(full_path, 'wb') as file:
        pickle.dump(trajectory_data, file)

def save_trajectory_to_csv(trajectory_data, filename):
    dir_path = "/home/cam/multi_iiwa_ws/src/moveit_motion/moveit_motion/robot_trajectories"
    full_path = os.path.join(dir_path, filename)
    with open(full_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_from_start'] + [f'joint_{i+1}' for i in range(len(trajectory_data.points[0].positions))])
        for point in trajectory_data.points:
            writer.writerow([point.time_from_start.to_sec()] + list(point.positions))

class MoveGroupActionClientNode(Node):
    def __init__(self, node_name, move_group_to_run="arm"):
        super().__init__(node_name)

        self.action_server = "move_action"
        self.execute_action_name_ = "execute_trajectory"
        self.plan_srv_name_ = "plan_kinematic_path"
        self.timeout_sec_ = 3.0
        self.move_group_name = move_group_to_run
        self.base = f"{move_group_to_run}_link_0"
        self.end_effector = f"{move_group_to_run}_link_ee"

        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        if not self.ik_client.wait_for_service(timeout_sec=1):
            raise RuntimeError(f"Couldn't connect to service {self.ik_client.srv_name}.")

        self.plan_client_ = self.create_client(GetMotionPlan, self.plan_srv_name_)
        if not self.plan_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error("Plan service not available.")
            exit(1)

        self.execute_client_ = ActionClient(self, ExecuteTrajectory, self.execute_action_name_)
        if not self.execute_client_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error("Execute action not available.")
            exit(1)

        self.total_trajectory = JointTrajectory()

    def request_inverse_kinematics(self, pose: Pose) -> JointState:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.move_group_name
        request.ik_request.pose_stamped.header.frame_id = self.base
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.avoid_collisions = True
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("Inverse kinematics service call failed")
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Failed to compute inverse kinematics: {response.error_code.val}")
            return None
        return response.solution.joint_state

    def sum_of_square_diff(self, joint_state_1: JointState, joint_state_2: JointState) -> float:
        return np.sum(np.square(np.subtract(joint_state_1.position, joint_state_2.position)))

    def get_best_ik(self, target_pose: Pose, attempts: int = 100) -> JointState | None:
        current_joint_state_set, current_joint_state = wait_for_message(
            JointState, self, f"/{self.move_group_name}/joint_states", time_to_wait=3.0
        )
        if not current_joint_state_set:
            self.get_logger().error("Failed to get current joint state")
            return None

        best_cost = np.inf
        best_joint_state = None

        for _ in range(attempts):
            joint_state = self.request_inverse_kinematics(target_pose)
            if joint_state is None:
                continue
            cost = self.sum_of_square_diff(current_joint_state, joint_state)
            if cost < best_cost:
                best_cost = cost
                best_joint_state = joint_state

        if not best_joint_state:
            self.get_logger().error("Failed to get IK solution")
        return best_joint_state

    def plan_to_joint_pos(self, target_joint_state=None, current_robot_state=None, attempts=1):
        if target_joint_state is None:
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
        request.motion_plan_request.group_name = self.move_group_name
        request.motion_plan_request.start_state = current_robot_state
        request.motion_plan_request.goal_constraints.append(target_constraint)
        request.motion_plan_request.num_planning_attempts = 10
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.05
        request.motion_plan_request.max_acceleration_scaling_factor = 0.1

        for attempt in range(attempts):
            plan_future = self.plan_client_.call_async(request)
            rclpy.spin_until_future_complete(self, plan_future)
            if plan_future.result() is None:
                self.get_logger().error("Failed to get motion plan")
                continue
            response = plan_future.result()
            if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f"Failed to get motion plan: {response.motion_plan_response.error_code.val}")
            else:
                return response.motion_plan_response.trajectory.joint_trajectory
        return None

    def execute_joint_traj(self, trajectory):

        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = self.total_trajectory
        goal_future = self.execute_client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Failed to execute trajectory")
            return None
        self.get_logger().info("Trajectory accepted, moving the robot...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future

    def get_current_robot_joint_state(self):
        _MSG_RECEIVED_BOOL, current_joint_state = wait_for_message(
            JointState, self, f"/{self.move_group_name}/joint_states", time_to_wait=3.0
        )
        if not _MSG_RECEIVED_BOOL:
            self.get_logger().error("Failed to get current joint state")
            return None
        return current_joint_state

    def modify_joint_state_for_sim_robot(self, robot_joint_state):
        _prefix_to_add = f"{self.move_group_name}"
        _name_of_all_joints = robot_joint_state.name
        for i in range(len(_name_of_all_joints)):
            _current_joint_name = _name_of_all_joints[i]
            _new_joint_name = f"{_prefix_to_add}_{_current_joint_name}"
            _name_of_all_joints[i] = _new_joint_name
        robot_joint_state.header.frame_id = "world"
        return robot_joint_state

    def connect_2_real_robot(self):
        _follow_joint_trajectory_for_move_group = f"/{self.move_group_name}/joint_trajectory_controller/follow_joint_trajectory"
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, _follow_joint_trajectory_for_move_group)
        if not self.trajectory_client.wait_for_server(timeout_sec=3):
            raise RuntimeError(f"Couldn't connect to Action Server {_follow_joint_trajectory_for_move_group}.")

    def execute_trajectory_on_real_robot(self, joint_trajectory: JointTrajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory
        goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return None
        self.get_logger().info("Trajectory goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future

def main():
    rclpy.init()

    move_group_action_client_node_sim_blue = MoveGroupActionClientNode(
        node_name="move_group_action_client_node_sim_blue",
        move_group_to_run="kuka_blue"
    )
    move_group_action_client_node_sim_green = MoveGroupActionClientNode(
        node_name="move_group_action_client_node_sim_green",
        move_group_to_run="kuka_green"
    )
    move_group_action_client_node_sim_dual = MoveGroupActionClientNode(
        node_name="move_group_action_client_node_sim_dual",
        move_group_to_run="dual_arm"
    )

    poses_blue = [
        Pose(
            position=Point(x=0.053813, y=-0.0006736, z=1.1839),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    ]

    kuka_blue_current_joint_state = move_group_action_client_node_sim_blue.get_current_robot_joint_state()
    kuka_blue_current_joint_state = move_group_action_client_node_sim_blue.modify_joint_state_for_sim_robot(kuka_blue_current_joint_state)

    kuka_green_current_joint_state = move_group_action_client_node_sim_green.get_current_robot_joint_state()
    kuka_green_current_joint_state = move_group_action_client_node_sim_green.modify_joint_state_for_sim_robot(kuka_green_current_joint_state)

    current_state_dual_inverse_list = JointState()
    current_state_dual_inverse_list.effort = kuka_blue_current_joint_state.effort
    current_state_dual_inverse_list.header = kuka_blue_current_joint_state.header
    current_state_dual_inverse_list.name = kuka_blue_current_joint_state.name
    current_state_dual_inverse_list.position = kuka_blue_current_joint_state.position + kuka_green_current_joint_state.position
    current_state_dual_inverse_list.velocity = kuka_blue_current_joint_state.velocity
    current_robot_state = RobotState()
    current_robot_state.joint_state = current_state_dual_inverse_list


    dual_spline_trajectory = []

    for pose in poses_blue:
        inverse_blue = move_group_action_client_node_sim_blue.get_best_ik(pose)
        inverse_green = move_group_action_client_node_sim_green.get_best_ik(pose)
        
        if inverse_blue is None or inverse_green is None:
            print("IK not Found for current pose : exiting the main function")
            return

        dual_inverse_list = JointState()
        dual_inverse_list.effort = inverse_blue.effort
        dual_inverse_list.header = inverse_blue.header
        dual_inverse_list.name = inverse_blue.name
        dual_inverse_list.position = inverse_blue.position[:7] + inverse_green.position[7:]
        dual_inverse_list.velocity = inverse_blue.velocity

        planned_joint_trajectory = move_group_action_client_node_sim_dual.plan_to_joint_pos(
            target_joint_state=dual_inverse_list,
            current_robot_state=current_robot_state
        )
        if planned_joint_trajectory is None:
            print("no planned trajectory")
            return
        
        dual_spline_trajectory.append(planned_joint_trajectory)

        # move_group_action_client_node_sim_dual.execute_joint_traj(planned_joint_trajectory)
    
    total_trajectory = JointTrajectory()
    rclpy.shutdown()
    
    '''
    green_traj_points = []
    blue_traj_points = []
    green_spline_trajectory = []
    blue_spline_trajectory = []

    for point in planned_joint_trajectory.points:
        green_waypoint = JointTrajectoryPoint()
        blue_waypoint = JointTrajectoryPoint()
        robot_dof = int(len(point.positions) / 2)
        green_waypoint.positions = point.positions[robot_dof:]
        blue_waypoint.positions = point.positions[:robot_dof]
        green_waypoint.accelerations = point.accelerations[robot_dof:]
        blue_waypoint.accelerations = point.accelerations[:robot_dof]
        green_waypoint.velocities = point.velocities[robot_dof:]
        blue_waypoint.velocities = point.velocities[:robot_dof]
        green_waypoint.effort = point.effort
        blue_waypoint.effort = point.effort
        green_waypoint.time_from_start = point.time_from_start
        blue_waypoint.time_from_start = point.time_from_start
        green_traj_points.append(green_waypoint)
        blue_traj_points.append(blue_waypoint)

    if not total_trajectory.points:
        last_time = 0.0
    else:
        last_point = total_trajectory.points[-1]
        last_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9

    for point in trajectory.points:
        new_point = JointTrajectoryPoint()
        new_point.positions = point.positions
        new_point.velocities = point.velocities
        new_point.accelerations = point.accelerations
        new_point.effort = point.effort
        new_point.time_from_start.sec = point.time_from_start.sec + int(last_time)
        new_point.time_from_start.nanosec = point.time_from_start.nanosec + int((last_time - int(last_time)) * 1e9)
        self.total_trajectory.points.append(new_point)

    current_state_dual_inverse_list = dual_inverse_list

    green_joint_traj = JointTrajectory()
    green_joint_traj.header.stamp.sec = 0
    green_joint_traj.header.stamp.nanosec = 0
    green_joint_traj.header.frame_id = ''
    green_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
    green_joint_traj.points = green_traj_points

    blue_joint_traj = JointTrajectory()
    blue_joint_traj.header.stamp.sec = 0
    blue_joint_traj.header.stamp.nanosec = 0
    blue_joint_traj.header.frame_id = ''
    blue_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
    blue_joint_traj.points = blue_traj_points

    save_trajectory(blue_joint_traj, "kuka_blue.dump")
    save_trajectory(green_joint_traj, "kuka_green.dump")
    save_trajectory_to_csv(blue_joint_traj, "kuka_blue.csv")
    save_trajectory_to_csv(green_joint_traj, "kuka_green.csv")
    '''


if __name__ == '__main__':
    main()
