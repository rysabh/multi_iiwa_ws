import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from typing import Optional  

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
)
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory

from submodules.wait_for_message import wait_for_message

import numpy as np

from submodules.RS_submodules import MSE_joint_states, filter_joints_for_move_group_name

from builtin_interfaces.msg import Duration

class MoveitInterface(Node):
    ik_client_name_ = "compute_ik"
    fk_srv_name_ = "compute_fk"
    plan_srv_name_ = "plan_kinematic_path"
    execute_action_name_ = "execute_trajectory"
    # action_server_ = "move_action" 

    def __init__(self, node_name, move_group_name="arm"):
        super().__init__(node_name)
        self.timeout_sec_ = 3.0
        self.move_group_name_ = move_group_name

        self.base_ = f"{self.move_group_name_}_link_0"
        self.end_effector_ = f"{self.move_group_name_}_link_ee"

        self.ik_client_ = self.create_client(GetPositionIK, self.ik_client_name_)
        if not self.ik_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: IK service not available -> {self.ik_client_.srv_name}.")
            exit(1)

        self.fk_client_ = self.create_client(GetPositionFK, self.fk_srv_name_)
        if not self.fk_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: FK service not available -> {self.fk_client_.srv_name}.")
            exit(1)
        
        self.plan_client_ = self.create_client(GetMotionPlan, self.plan_srv_name_)
        if not self.plan_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: GetMotionPlan service not available -> {self.plan_client_.srv_name}.")
            exit(1)
        
        self.execute_client_ = ActionClient(self, ExecuteTrajectory, self.execute_action_name_)
        if not self.execute_client_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: ExecuteActionClient-Sim action not available -> {self.execute_client_._action_name}")
            exit(1)


    @filter_joints_for_move_group_name
    def get_current_joint_state(self) -> JointState | None: # use descriptors for filtering
        '''
        Joint State for the moveit move group
        '''
        _JOINT_STATE_TOPIC = "/joint_states"

        _MSG_RECEIVED_BOOL, _current_joint_state = wait_for_message(
            JointState, self, _JOINT_STATE_TOPIC, time_to_wait=self.timeout_sec_
        )
        if not _MSG_RECEIVED_BOOL:
            self.get_logger().error("Failed to get current joint state")
            return None
        
        return _current_joint_state

    def get_current_robot_pose(self) -> Pose | None:
        '''
        Pose for the real robot
        '''
        _current_joint_state = self.get_current_joint_state()
        _current_robot_state = RobotState()
        _current_robot_state.joint_state = _current_joint_state

        _request = GetPositionFK.Request()
        _request.header.frame_id = f"{self.move_group_name_}/{self.base_}"  # TODO this has to be robot_name in the future
        _request.header.stamp = self.get_clock().now().to_msg()
        _request.fk_link_names.append(self.end_effector_)
        _request.robot_state = _current_robot_state

        future = self.fk_client_.call_async(_request)
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

    @filter_joints_for_move_group_name
    def request_ik(self, pose: Pose) -> JointState | None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.move_group_name_
        request.ik_request.pose_stamped.header.frame_id = self.base_
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.avoid_collisions = True
        future = self.ik_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error(f"Inverse kinematics service call failed for move group -> {self.move_group_name_}")
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Failed to compute inverse kinematics: {response.error_code.val}")
            return None
        ik_solution = response.solution.joint_state
        return  ik_solution
    
    def get_best_ik(self, current_joint_state:JointState, target_pose: Pose, attempts: int = 100) -> JointState | None:
        # if not current_pose:
        #     current_joint_state = self.get_current_joint_state()


        best_cost = np.inf
        best_joint_state = None

        for _ in range(attempts):
            target_joint_state = self.request_ik(target_pose)
            if target_joint_state is None:
                continue
            cost = MSE_joint_states(current_joint_state, target_joint_state)
            if cost < best_cost:
                best_cost = cost
                best_joint_state = target_joint_state

        if not best_joint_state:
            self.get_logger().error("Failed to get any IK solution")
        else:
            self.get_logger().info(f"joint state motion -> \n{np.round(current_joint_state.position,2)} -> {np.round(best_joint_state.position,2)}\n")
        return best_joint_state


    def get_joint_traj(self, target_joint_state: JointState,
                             start_joint_state: JointState, 
                       attempts: int = 10, **kwargs) -> RobotTrajectory | None:
        '''
        kwargs = {planner_type = "linear" | None,}
        ''' 
        ## Create start state using start_joint_state #TODO automate sim vs real
        # if start_joint_state is None:
        #     start_joint_state = self.get_robot_current_joint_state()

        # if start_joint_state is None:
        #     start_joint_state = self.get_current_joint_state()
        
        
        ## if start and target match exit the function
        _THRESHOLD = 0.01 # threshold from start and target for robot to move
        if MSE_joint_states(target_joint_state, start_joint_state) <= _THRESHOLD:
            self.get_logger().info("Start and End goals match. ** NOT ** moving anything and Passing Empty Trajectory")
            return RobotTrajectory()
        
        ##Create target using target_joint_state
        target_constraint = Constraints()
        for i in range(len(target_joint_state.name)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = target_joint_state.name[i]
            joint_constraint.position = target_joint_state.position[i]
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            target_constraint.joint_constraints.append(joint_constraint)
        
        ## convert start_joint_state to robot_state to get motion plan
        _start_robot_state = RobotState()
        _start_robot_state.joint_state = start_joint_state

        ## Request Motion Plan
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.move_group_name_
        request.motion_plan_request.start_state = _start_robot_state
        request.motion_plan_request.goal_constraints.append(target_constraint)
        request.motion_plan_request.num_planning_attempts = 10  #TODO -> earlier it was 10 -> how is this different from "attempts"
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.05
        request.motion_plan_request.max_acceleration_scaling_factor = 0.1
        
        ### set motion planner type
        planner_type = kwargs.get("planner_type")
        if planner_type == "linear":
            request.motion_plan_request.pipeline_id = "pilz_industrial_motion_planner"
            request.motion_plan_request.planner_id = "LIN"
        else:
            request.motion_plan_request.pipeline_id = "ompl"
            request.motion_plan_request.planner_id = "APSConfigDefault"

        ## plan for n attempts until succesful
        for _ in range(attempts):
            plan_future = self.plan_client_.call_async(request)
            rclpy.spin_until_future_complete(self, plan_future)
            if plan_future.result() is None:
                self.get_logger().error("Failed to get motion plan")
                continue
            response = plan_future.result()
            if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f"Failed to get motion plan: {response.motion_plan_response.error_code.val}")
            else:
                #response.motion_plan_response.trajectory -> RobotTrajectory
                #response.motion_plan_response.trajectory.joint_trajectory -> JointTrajectory
                # However execute_joint_traj needs RobotTrajectory anyways (for both sim -> ExecuteTrajectory.Goal()   and real -> FollowJointTrajectory.Goal() )
                return response.motion_plan_response.trajectory
        return None


    def execute_joint_traj(self, trajectory: RobotTrajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self.execute_client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        
        goal_handle = goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Failed to execute trajectory")
            return None
        
        self.get_logger().info("Trajectory accepted, moving the robot...")
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)
        #Hantao's Code
        # expect_duration = traj.joint_trajectory.points[-1].time_from_start
        # expect_time = time.time() + 2 * expect_duration.sec 
        # while not result_future.done() and time.time() < expect_time:
        #     time.sleep(0.01)

        # robot_interface_node.get_logger().info("Trajectory executed")
        if not result_future.done():
            self.get_logger().info("!!! Trajectory NOT executed")
            raise RuntimeError
        
        self.get_logger().info("Trajectory executed")
        return
        
    @staticmethod
    def combine_trajectories(robot_traj_1: RobotTrajectory, robot_traj_2: RobotTrajectory) -> RobotTrajectory:
        print("Combining Trajectories")
        # Calculate the duration of the first trajectory
        joint_traj_1 = robot_traj_1.joint_trajectory
        joint_traj_2 = robot_traj_2.joint_trajectory

        if joint_traj_1.points:
            t1 = joint_traj_1.points[-1].time_from_start
        else:
            t1 = Duration(sec=0, nanosec=0)
        
        # Adjust the time_from_start for joint_traj_2
        for point in joint_traj_2.points:
            point.time_from_start.sec += t1.sec
            point.time_from_start.nanosec += t1.nanosec
            # Normalize nanoseconds if necessary
            if point.time_from_start.nanosec >= 1e9:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec -= int(1e9)
        
        # Combine the points from both trajectories
        combined_points = joint_traj_1.points + joint_traj_2.points
        
        # Create a new JointTrajectory message
        combined_joint_trajectory = JointTrajectory()
        combined_joint_trajectory.header = joint_traj_1.header
        combined_joint_trajectory.joint_names = joint_traj_1.joint_names
        combined_joint_trajectory.points = combined_points
        combined_robot_trajectory = RobotTrajectory()
        combined_robot_trajectory.joint_trajectory = combined_joint_trajectory
        print("Trajectories Combined")
        return combined_robot_trajectory
