import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from typing import Optional, Union

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

from ros_submodules.wait_for_message import wait_for_message

import numpy as np

from ros_submodules.RS_submodules import MSE_joint_states, filter_for_prefix

from builtin_interfaces.msg import Duration
import os

class MoveitInterface(Node):
    '''
    MoveitInterface(
        node_name="client_blue",      # lbr / kuka_blue -> required for FK service
        move_group_name="kuka_blue",  # arm / kuka_blue -> required for motion planning
        remapping_name="",            # lbr / ""        -> required for service and action remapping
        prefix="kuka_blue",           # ""  / kuka_blue -> required for filtering joint states and links
    ) 
    '''
    def __init__(self, node_name, move_group_name="arm", remapping_name="lbr", prefix=""):
        super().__init__(node_name)
        self.node_name_ = node_name
        self.timeout_sec_ = 3.0
        self.move_group_name_ = move_group_name
        self.remapping_name_ = remapping_name
        self.prefix_ = prefix
        
        self.joint_states_topic_ = f"{remapping_name}/joint_states" if remapping_name else "joint_states"
        self.ik_srv_name_ = f"{remapping_name}/compute_ik" if remapping_name else "compute_ik"
        self.fk_srv_name_ = f"{remapping_name}/compute_fk" if remapping_name else "compute_fk"
        self.plan_srv_name_ = f"{remapping_name}/plan_kinematic_path" if remapping_name else "plan_kinematic_path"
        self.execute_action_name_ = f"{remapping_name}/execute_trajectory" if remapping_name else "execute_trajectory"
        # self.action_server_ = f"{remapping_name}/move_action" if remapping_name else "move_action"

        #link names
        self.base_ = f"{prefix}_link_0" if prefix else f"{remapping_name}/link_0"
        self.end_effector_ = f"{prefix}_link_ee" if prefix else f"{remapping_name}/link_ee"
        

        self.ik_client_ = self.create_client(GetPositionIK, self.ik_srv_name_)
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


    @filter_for_prefix
    def get_current_joint_state(self) -> Union[JointState, None]: # use descriptors for filtering
        '''
        Joint State for the moveit move group
        '''
        _JOINT_STATE_TOPIC = self.joint_states_topic_

        _MSG_RECEIVED_BOOL, _current_joint_state = wait_for_message(
            JointState, self, _JOINT_STATE_TOPIC, time_to_wait=self.timeout_sec_
        )
        if not _MSG_RECEIVED_BOOL:
            self.get_logger().error("Failed to get current joint state")
            return None
        
        return _current_joint_state

    def get_current_robot_pose(self) -> Union[Pose , None]:
        '''
        Pose for the real robot
        '''
        _current_joint_state = self.get_current_joint_state()
        _current_robot_state = RobotState()
        _current_robot_state.joint_state = _current_joint_state

        _request = GetPositionFK.Request()
        _request.header.frame_id = self.base_
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

    @filter_for_prefix
    def request_ik(self, pose: Pose) -> Union[JointState, None]:
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
    
    def get_best_ik(self, current_joint_state:JointState, target_pose: Pose, attempts: int = 100) -> Union[JointState, None]:
        # if not current_pose: #TODO -> check if this is necessary
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
                       attempts: int = 10, **kwargs) -> Union[RobotTrajectory, None]:
        '''
        kwargs = {planner_type = "linear" | None,}
        ''' 
        ## Create start state using start_joint_state #TODO automate sim vs real
        # if start_joint_state is None:
        #     start_joint_state = self.get_robot_current_joint_state()

        # if start_joint_state is None: #TODO -> check if this is necessary
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
        PLANNER_CONIFIG = {
            None  : "APSConfigDefault",
            "ompl": "RRTstar",
            "pilz": "pilz_industrial_motion_planner",
        }

        if planner_type not in PLANNER_CONIFIG.keys():
            self.get_logger().error(f"Invalid Planner Type: {planner_type}.\nValid options are:\n{PLANNER_CONIFIG}")
            exit(1)

        request.motion_plan_request.pipeline_id = planner_type
        request.motion_plan_request.planner_id = PLANNER_CONIFIG[planner_type]
        self.get_logger().info(f"Using {planner_type} Planner -> {PLANNER_CONIFIG[planner_type]}")

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
    def combine_trajectories(trajectories: list[RobotTrajectory]) -> RobotTrajectory:
        print("Combining Trajectories")
        
        if not trajectories:
            raise ValueError("The list of trajectories is empty.")
        
        # Initialize the combined trajectory with the first trajectory
        combined_trajectory = trajectories[0]
        combined_points = list(combined_trajectory.joint_trajectory.points)
        
        # Iterate over the remaining trajectories
        current_time = combined_points[-1].time_from_start if combined_points else Duration(sec=0, nanosec=0)
        
        for robot_traj in trajectories[1:]:
            joint_trajectory = robot_traj.joint_trajectory
            for point in joint_trajectory.points:
                # Adjust time_from_start for each point in the current trajectory
                point.time_from_start.sec += current_time.sec
                point.time_from_start.nanosec += current_time.nanosec
                if point.time_from_start.nanosec >= 1e9:
                    point.time_from_start.sec += 1
                    point.time_from_start.nanosec -= int(1e9)
            
            # Append points, excluding the first point to avoid duplicating the end/start point
            combined_points.extend(joint_trajectory.points[1:])
            
            # Update current time for the next trajectory
            current_time = combined_points[-1].time_from_start if combined_points else Duration(sec=0, nanosec=0)
        
        # Create the final combined JointTrajectory message
        combined_joint_trajectory = JointTrajectory()
        combined_joint_trajectory.header = combined_trajectory.joint_trajectory.header
        combined_joint_trajectory.joint_names = combined_trajectory.joint_trajectory.joint_names
        combined_joint_trajectory.points = combined_points
        
        # Create the final combined RobotTrajectory message
        combined_robot_trajectory = RobotTrajectory()
        combined_robot_trajectory.joint_trajectory = combined_joint_trajectory
        
        print("Trajectories Combined")
        return combined_robot_trajectory
