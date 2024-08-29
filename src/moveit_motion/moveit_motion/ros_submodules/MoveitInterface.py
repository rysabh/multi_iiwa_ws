import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from typing import Optional, Union

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
)
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK, GetCartesianPath

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory

from moveit_motion.ros_submodules.wait_for_message import wait_for_message

import numpy as np

from moveit_motion.ros_submodules.RS_submodules import MSE_joint_states, filter_for_prefix
import moveit_motion.ros_submodules.ros_math as rosm


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
    PLANNER_CONFIG = {
        "default": ["ompl", "APSConfigDefault"],
        "ompl": ["ompl", "APSConfigDefault"],
        "pilz": ["pilz_industrial_motion_planner", "LIN"]
        }
    
    THRESHOLD_2_MOVE = 0.0005 # 0.0005 
    
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
        self.cartesian_srv_name_ = f"{remapping_name}/compute_cartesian_path" if remapping_name else "compute_cartesian_path"
        # self.action_server_ = f"{remapping_name}/move_action" if remapping_name else "move_action"

        #link names 
        self.base_ = f"{prefix}_link_0" if prefix else f"{remapping_name}/link_0" # for FK and IK
        self.end_effector_ = f"{prefix}_link_ee" if prefix else 'link_ee'    # for FK
        

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
            
        self.cartesian_client_ = self.create_client(GetCartesianPath, self.cartesian_srv_name_)
        if not self.cartesian_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: GetCartesianPath service not available -> {self.cartesian_client_.srv_name}.")
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
        _current_robot_state = rosm.joint_2_robot_state(_current_joint_state)

        _request = GetPositionFK.Request()
        _request.header.frame_id = self.base_
        # _request.header.frame_id = 'lbr/link_0' #kuka_blue_link_0
        _request.header.stamp = self.get_clock().now().to_msg()
        # _request.fk_link_names.append('link_ee')
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
            self.get_logger().info(f"IK found for -> \n{np.round(current_joint_state.position,2)} -> {np.round(best_joint_state.position,2)}\n")
        return best_joint_state


    
    def _get_planner_config(self, planner_type: str) -> tuple:
        if planner_type not in self.PLANNER_CONFIG.keys():
            self.get_logger().error(f"Invalid Planner Type: {planner_type}.\nValid options are: {list(self.PLANNER_CONFIG.keys())}")
            exit(1)
        self.get_logger().info(f"Using {planner_type} Planner -> {self.PLANNER_CONFIG[planner_type]}")
        return self.PLANNER_CONFIG[planner_type]
    

    def _create_motion_plan_request(self, start_state: RobotState, goal_constraints: Constraints, **kwargs) -> GetMotionPlan.Request:
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = self.move_group_name_
        request.motion_plan_request.start_state = start_state
        request.motion_plan_request.goal_constraints.append(goal_constraints)
        request.motion_plan_request.num_planning_attempts = kwargs.get("attempts", 10) #TODO -> earlier it was 10 -> how is this different from "attempts"
        request.motion_plan_request.allowed_planning_time = kwargs.get("allowed_planning_time", 5.0)
        request.motion_plan_request.max_velocity_scaling_factor = kwargs.get("max_velocity_scaling_factor", 0.05)
        request.motion_plan_request.max_acceleration_scaling_factor = kwargs.get("max_acceleration_scaling_factor", 0.1)
        request.motion_plan_request.pipeline_id = kwargs.get("pipeline_id", "ompl")
        request.motion_plan_request.planner_id = kwargs.get("planner_id", "APSConfigDefault")
        # self.get_logger().info(f"requesting plan for -> \n{np.round(start_state.joint_state.position,2)} -> {np.round(goal_constraints.joint_constraints[-1].joint_state.position,2)}\n")
        return request
    
    
    def _request_for_attempts(self, request, client, response_handler, attempts: int) -> Union[RobotTrajectory, None]:
        for _ in range(attempts):
            plan_future = client.call_async(request)
            rclpy.spin_until_future_complete(self, plan_future)
            if plan_future.result() is None:
                self.get_logger().error("Failed to get response from service")
                continue
            response = plan_future.result()
            trajectory = response_handler(response)
            if trajectory:
                return trajectory
        return None
    

    def _filter_waypoints_threshold(self, waypoints: list[JointState]) -> list[JointState]:
        filtered_waypoints = [waypoints[0]]
        for i in range(1, len(waypoints)):
            if MSE_joint_states(waypoints[i], waypoints[i-1]) > self.THRESHOLD_2_MOVE:
                filtered_waypoints.append(waypoints[i])
        return filtered_waypoints

    #===================================================================
    ###################### MOVEIT EXECUTION ############################
    #===================================================================
     
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
    #================== End of Moveit Execution ========================

    #===================================================================
    ###################### MOVEIT PLANNING ############################
    #===================================================================

    ###---------------------- Joint Space Planning -----------------------
    def get_joint_ptp_plan(self, start_joint_state: JointState, target_joint_state: JointState, attempts: int = 100, **kwargs) -> Union[RobotTrajectory, None]:
        if MSE_joint_states(target_joint_state, start_joint_state) <= self.THRESHOLD_2_MOVE:
            self.get_logger().info("Start and End goals match. ** NOT ** moving anything and Passing Empty Trajectory")
            return RobotTrajectory()
        
        ##Create target using target_joint_state
        _constraints = rosm.joint_states_2_constraints(target_joint_state)
        
        ## convert start_joint_state to robot_state to get motion plan
        _start_robot_state = rosm.joint_2_robot_state(start_joint_state)

        self.get_logger().info(f"requesting plan for -> \n{np.round(start_joint_state.position,2)} -> {np.round(target_joint_state.position,2)}\n")

        ### set motion planner type
        planner_type = kwargs.get("planner_type", "default")
        pipeline_id, planner_id = self._get_planner_config(planner_type)
        
        ## Request Motion Plan
        request = self._create_motion_plan_request(_start_robot_state, _constraints, attempts=attempts,
                                         pipeline_id=pipeline_id, planner_id=planner_id, **kwargs)

        def handle_joint_response(response):
            if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f"Failed to get motion plan: {response.motion_plan_response.error_code.val}")
                return None
            return response.motion_plan_response.trajectory
        
        ## plan for n attempts until succesful
        client = self.plan_client_

        return self._request_for_attempts(request, client, handle_joint_response, attempts)
    
    
    def get_joint_spline_plan(self, waypoints: list[JointState], attempts: int = 100, **kwargs) -> Union[RobotTrajectory, None]:
        waypoints = self._filter_waypoints_threshold(waypoints)
        if len(waypoints) < 2:
            self.get_logger().info("Not enough waypoints to move. ** NOT ** moving anything and Passing Empty Trajectory")
            return RobotTrajectory()
        
        _start_joint_state = waypoints[0]
        _start_robot_state = rosm.joint_2_robot_state(_start_joint_state)
          
        _constraints = rosm.joint_states_2_constraints(*waypoints[1:])
        

        planner_type = kwargs.get("planner_type", "default")
        pipeline_id, planner_id = self._get_planner_config(planner_type)

        request = self._create_motion_plan_request(_start_robot_state, _constraints, attempts=attempts,
                                         pipeline_id=pipeline_id, planner_id=planner_id, **kwargs)
        ## plan for n attempts until succesful
        return self._request_plan_for_attempts(request, attempts)
    
    ###---------------------- Cartesian Space Planning -----------------------
    def get_cartesian_ptp_plan():
        pass

    def get_cartesian_spline_plan(self, waypoints: list[Pose], attempts:int = 100, eef_step: float = 0.01, jump_threshold: float = 0.0, avoid_collisions: bool = False) -> Union[RobotTrajectory, None]:
        '''
        Plan a Cartesian path (spline) through the given waypoints.
        
        :param waypoints: List of Pose objects defining the Cartesian waypoints.
        :param eef_step: The step size for end effector in Cartesian space.
        :param jump_threshold: The threshold for allowed joint space jumps.
        :param avoid_collisions: Whether to avoid collisions while planning.
        :return: Planned Cartesian path as a RobotTrajectory object, or None if planning failed.
        '''
        # Create request for Cartesian path
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = self.move_group_name_
        request.link_name = self.end_effector_
        # request.waypoints = [PoseStamped(header=request.header, pose=wp) for wp in waypoints]
        request.waypoints = waypoints
        request.max_step = eef_step
        request.jump_threshold = jump_threshold
        request.avoid_collisions = avoid_collisions

        def handle_cartesian_response(response):
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f"Failed to compute Cartesian path: {response.error_code.val}")
                return None
            if response.fraction < 1.0:
                self.get_logger().error(f"===== Fraction achieved ====== {response.fraction}")
                return None
            return response.solution

        # Call the service and wait for response
        client = self.cartesian_client_
        return self._request_for_attempts(request, client, handle_cartesian_response, attempts=attempts)
    

    #================== End of Moveit Planning ========================