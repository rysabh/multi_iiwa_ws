import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from typing import Optional, Union
from std_msgs.msg import Header
from moveit_msgs.action import (
    MoveGroup,
    ExecuteTrajectory,
    MoveGroupSequence
)

from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,

    BoundingVolume,
    MotionPlanRequest,
    MotionSequenceItem,
    MotionSequenceRequest,
    OrientationConstraint,
    PositionConstraint,
    WorkspaceParameters
)

from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import (
    GetPositionIK, 
    GetMotionPlan, 
    GetPositionFK, 
    GetCartesianPath, 
    GetMotionSequence
    )

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory

from moveit_motion.ros_submodules.wait_for_message import wait_for_message

import numpy as np
import moveit_motion.ros_submodules.RS_submodules as rsmod
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
        "pilz": ["pilz_industrial_motion_planner", "LIN"],
        "cartesian": []
        }
    
    THRESHOLD_2_MOVE = 0.000005 # 0.0005

    CARTESIAN_SERVICE_CLIENT_FLAG = False
    EXECUTE_ACTION_CLIENT_FLAG = False
    SEQUENCE_CLIENT_FLAG = False
    
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
        
        # self.action_server_ = f"{remapping_name}/move_action" if remapping_name else "move_action"

        #link names 
        self.base_ = f"{prefix}_link_0" if prefix else f"{remapping_name}/link_0" # for FK and IK
        # self.end_effector_ = f"{prefix}_link_ee" if prefix else 'link_ee'    # for FK
        self.end_effector_ = f"{prefix}_link_tcp" if prefix else 'link_tcp'    # for FK
        

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
        

    @rsmod.filter_joint_state_for_prefix
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
        
        return rsmod.sort_joint_state(_current_joint_state)

    def get_fk(self, joint_state: JointState) -> Union[Pose, None]:
        '''
        Pose for the real robot
        '''
        _robot_state = rosm.joint_2_robot_state(joint_state)
        _request = GetPositionFK.Request()
        # _request.header.frame_id = self.base_
        _request.header.frame_id = 'world'
        # _request.header.frame_id = 'lbr/link_0' #kuka_blue_link_0
        _request.header.stamp = self.get_clock().now().to_msg()
        # _request.fk_link_names.append('link_ee')
        _request.fk_link_names.append(self.end_effector_)
        _request.robot_state = _robot_state

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

    def get_current_robot_pose(self) -> Union[Pose , None]:
        '''
        Current Pose for the real robot
        '''
        _current_joint_state = self.get_current_joint_state()
        if not _current_joint_state:
            return None
        return self.get_fk(_current_joint_state)

    #TODO - add planning frame to get IK
    @rsmod.filter_joint_state_for_prefix
    def get_ik(self, pose: Pose) -> Union[JointState, None]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.move_group_name_
        # request.ik_request.pose_stamped.header.frame_id = self.base_
        request.ik_request.pose_stamped.header.frame_id = "world"
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
            target_joint_state = self.get_ik(target_pose)
            if target_joint_state is None:
                continue
            cost = rsmod.MSE_joint_states(current_joint_state, target_joint_state)
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
    
    
    def _request_srv_for_attempts(self, request, client, response_handler, attempts: int) -> Union[RobotTrajectory, None]:
        for _ in range(attempts):
            plan_future = client.call_async(request)
            rclpy.spin_until_future_complete(self, plan_future)
            if plan_future.result() is None:
                self.get_logger().error("Failed to get response from service")
                continue
            response = plan_future.result()
            _response_handle = response_handler(response)
            
            if _response_handle['stop_flag']:
                return _response_handle
            
            self.get_logger().error("Failed to get any plan")
        return None
    
    def _request_action_for_attempts(self, request, client, response_handler, attempts: int) -> Union[RobotTrajectory, None]:
        for _ in range(attempts):
            plan_future = client.send_goal_async(request)
            rclpy.spin_until_future_complete(self, plan_future)
            if plan_future.result() is None:
                self.get_logger().error("Failed to get response from Action")
                continue
            response = plan_future.result()
            _response_handle = response_handler(response)
            
            if _response_handle['stop_flag']:
                return _response_handle
            
            self.get_logger().error("Failed to get any plan")
        return None

    def _filter_waypoints_threshold(self, waypoints: list[JointState]) -> list[JointState]:
        filtered_waypoints = [waypoints[0]]
        for i in range(1, len(waypoints)):
            if rsmod.MSE_joint_states(waypoints[i], waypoints[i-1]) > self.THRESHOLD_2_MOVE:
                filtered_waypoints.append(waypoints[i])
        return filtered_waypoints

    #===================================================================
    ###################### MOVEIT EXECUTION ############################
    #===================================================================
    def _set_execute_action_client(self, action_name: str = "execute_trajectory") -> None:
        self.execute_action_name_ = f"{self.remapping_name_}/{action_name}" if self.remapping_name_ else action_name
        self.execute_client_ = ActionClient(self, ExecuteTrajectory, self.execute_action_name_)
        if not self.execute_client_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: ExecuteActionClient-Sim action not available -> {self.execute_client_._action_name}")
            exit(1)
        self.EXECUTE_ACTION_CLIENT_FLAG = True

    def execute_joint_traj(self, trajectory: RobotTrajectory):
        self._set_execute_action_client()
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

        # rclpy.spin_until_future_complete(self, result_future)
        #Hantao's Code
        # expect_duration = traj.joint_trajectory.points[-1].time_from_start
        # expect_time = time.time() + 2 * expect_duration.sec 
        # while not result_future.done() and time.time() < expect_time:
        #     time.sleep(0.01)

        # robot_interface_node.get_logger().info("Trajectory executed")
        # if not result_future.done():
        #     self.get_logger().info("!!! Trajectory NOT executed")
        #     raise RuntimeError
        
        self.get_logger().info("Trajectory executed")
        return
    #================== End of Moveit Execution ========================

    #===================================================================
    ###################### MOVEIT PLANNING ############################
    #===================================================================

    ###---------------------- Joint Space Planning -----------------------
    # def _create_motion_plan_request(self, goal_constraints: Constraints, 
    #                                 start_state: RobotState = None, 
    #                                 **kwargs) -> GetMotionPlan.Request:
    #     request = GetMotionPlan.Request()
    #     request.motion_plan_request.group_name = self.move_group_name_
    #     # request.motion_plan_request.start_state = start_state
    #     request.motion_plan_request.goal_constraints.append(goal_constraints)
    #     request.motion_plan_request.num_planning_attempts = kwargs.get("attempts", 10) #TODO -> earlier it was 10 -> how is this different from "attempts"
    #     request.motion_plan_request.allowed_planning_time = kwargs.get("allowed_planning_time", 5.0)
    #     request.motion_plan_request.max_velocity_scaling_factor = kwargs.get("max_velocity_scaling_factor", 0.05)
    #     request.motion_plan_request.max_acceleration_scaling_factor = kwargs.get("max_acceleration_scaling_factor", 0.1)
    #     request.motion_plan_request.pipeline_id = kwargs.get("pipeline_id", "ompl")
    #     request.motion_plan_request.planner_id = kwargs.get("planner_id", "APSConfigDefault")
    #     # self.get_logger().info(f"requesting plan for -> \n{np.round(start_state.joint_state.position,2)} -> {np.round(goal_constraints.joint_constraints[-1].joint_state.position,2)}\n")
    #     return request
    
    
    def _create_motion_plan_request(self, goal_constraints: list[Constraints],
                                    path_constraints: list[Constraints] = None,
                                    start_joint_state: JointState = None, 
                                    **kwargs) -> GetMotionPlan.Request:
        
        request = GetMotionPlan.Request(
            motion_plan_request=MotionPlanRequest(
                workspace_parameters=WorkspaceParameters(),  #header, min_corner, max_corner
                group_name=self.move_group_name_,
                # start_state=rosm.joint_2_robot_state(start_joint_state if start_joint_state else self.get_current_joint_state()),
                # start_state=rosm.joint_2_robot_state(start_joint_state),
                goal_constraints=goal_constraints,
                # path_constraints=path_constraints if path_constraints else [],  # Ensure it's a list,
                num_planning_attempts=kwargs.get("attempts", 10),
                allowed_planning_time=kwargs.get("allowed_planning_time", 5.0),
                max_velocity_scaling_factor=kwargs.get("max_velocity_scaling_factor", 0.05),
                max_acceleration_scaling_factor=kwargs.get("max_acceleration_scaling_factor", 0.1),
                pipeline_id=kwargs.get("pipeline_id", "ompl"),
                planner_id=kwargs.get("planner_id", "APSConfigDefault")
                )
            )
        
        # self.get_logger().info(f"requesting plan for -> \n{np.round(start_joint_state.position,2)} -> {np.round(goal_constraints[-1].joint_constraints[-1].position,2)}\n")
        return request

    def _motion_plan_response_handler(self, response):
        _response_handle = {'trajectory': None, 'stop_flag': False}

        if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Failed to get motion plan: {response.motion_plan_response.error_code.val}")
            return _response_handle
        
        _response_handle['trajectory'] = response.motion_plan_response.trajectory
        _response_handle['stop_flag'] = True
        return _response_handle

    
    def get_joint_ptp_plan(self, start_joint_state: JointState, target_joint_state: JointState, attempts: int = 100, **kwargs) -> Union[RobotTrajectory, None]:
        if rsmod.MSE_joint_states(target_joint_state, start_joint_state) <= self.THRESHOLD_2_MOVE:
            self.get_logger().info("Start and End goals match. ** NOT ** moving anything and Passing Empty Trajectory")
            return RobotTrajectory()
        
        ##Create target using target_joint_state
        _constraints = [rosm.joint_states_2_constraints(target_joint_state)]
        
        self.get_logger().info(f"requesting plan for -> \n{np.round(start_joint_state.position,2)} -> {np.round(target_joint_state.position,2)}\n")

        ### set motion planner type
        planner_type = kwargs.get("planner_type", "default")
        pipeline_id, planner_id = self._get_planner_config(planner_type)
        
        ## Request Motion Plan
        # request = self._create_motion_plan_request(_start_robot_state, _constraints, attempts=attempts,
        #                                  pipeline_id=pipeline_id, planner_id=planner_id, **kwargs)
        
        request = self._create_motion_plan_request(goal_constraints=_constraints,
                                                   start_joint_state=start_joint_state, 
                                                   attempts=attempts,
                                                   pipeline_id=pipeline_id, planner_id=planner_id, 
                                                   **kwargs)
        ## plan for n attempts until succesful
        return self._request_srv_for_attempts(request, self.plan_client_, self._motion_plan_response_handler, attempts)
    
    
    #TODO - Later
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
        return self._request_srv_for_attempts(request, attempts)
    
    ### ---------------------- Cartesian Space Planning -----------------------
    def get_cartesian_ptp_plan(self, start_pose: Pose, target_pose: Pose, attempts: int = 100, **kwargs) -> Union[RobotTrajectory, None]:
        current_joint_state = self.get_current_joint_state()
        start_joint_state = self.get_best_ik(current_joint_state = current_joint_state, target_pose=start_pose, attempts=attempts)
        target_joint_state = self.get_best_ik(current_joint_state = start_joint_state, target_pose=target_pose, attempts=attempts)
        return self.get_joint_ptp_plan(start_joint_state, target_joint_state, attempts=attempts, **kwargs)

    
    def get_cartesian_spline_plan(self,
                                  waypoints: list[Pose], 
                                  planning_frame: str,
                                  attempts:int = 100,
                                  _planner_type = "cartesian_interpolator",
                                  **kwargs) -> Union[RobotTrajectory, None]:
        '''
        Plan a Cartesian path (spline) through the given waypoints.
        
        :param waypoints: List of Pose objects defining the Cartesian waypoints.
        :param planning_frame: The frame in which the waypoints are defined. -> self.base_ or 'world'
        :param eef_step: The step size for end effector in Cartesian space.
        :param jump_threshold: The threshold for allowed joint space jumps.
        :param avoid_collisions: Whether to avoid collisions while planning.
        :return: Planned Cartesian path as a RobotTrajectory object, or None if planning failed.

        **kwargs -> cartesian planner specific arguments:
        max_step: float = 0.01, jump_threshold: float = 0.0, avoid_collisions: bool = False
        
        **kwargs -> pilz planner specific arguments:
        max_velocity_scaling_factor: float = 0.1, max_acceleration_scaling_factor: float = 0.1
        '''
        _CHOICES = {
                'cartesian_interpolator': {
                'create_client': self._set_cartesian_interpolator_service_client,
                'create_request': self._create_cartesian_interpolator_motion_plan_request,
                "requester": self._request_srv_for_attempts,
                'response_handler': self._cartesian_interpolator_response_handler,
                'SETUP_FLAG': self.CARTESIAN_SERVICE_CLIENT_FLAG
            },
                'cartesian_sequence': {
                'create_client': self._set_cartesian_sequence_service_client,
                'create_request': self._create_motion_sequence_request,
                'requester': self._request_srv_for_attempts,
                'response_handler': self._motion_sequence_response_handler,
                'SETUP_FLAG': self.SEQUENCE_CLIENT_FLAG
            },
                'cartesian_sequence_action': {
                'create_client': self._set_cartesian_sequence_action_client,
                'create_request': self._create_motion_sequence_goal_request,
                'requester': self._request_action_for_attempts,
                'response_handler': self._motion_sequence_action_response_handler,
                'SETUP_FLAG': self.SEQUENCE_CLIENT_FLAG
            },

        }

        if _planner_type not in _CHOICES.keys():
            self.get_logger().error(f"Invalid Planner Type: {_planner_type}.\nValid options are: {list(_CHOICES.keys())}")
            exit(1)
            
        if not _CHOICES[_planner_type]['SETUP_FLAG']:
            _CHOICES[_planner_type]['create_client']()

        _request = _CHOICES[_planner_type]['create_request'](waypoints, planning_frame, **kwargs)
        _requester = _CHOICES[_planner_type]['requester']
        _response_handle = _requester(_request, self.spline_client_, _CHOICES[_planner_type]['response_handler'], attempts=attempts)
        return _response_handle

    
    
    ### TODO - Doing (Read chatGPT)
    def _set_cartesian_interpolator_service_client(self, service_name: str = "compute_cartesian_path") -> None:
        self.spline_srv_name_ = f"{self.remapping_name_}/{service_name}" if self.remapping_name_ else service_name    
        self.spline_client_ = self.create_client(GetCartesianPath, self.spline_srv_name_)
        if not self.spline_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: GetCartesianPath service not available -> {self.spline_client_.srv_name}.")
            exit(1)
        self.CARTESIAN_SERVICE_CLIENT_FLAG = True

    def _create_cartesian_interpolator_motion_plan_request(self, waypoints: list[Pose], 
                                                           planning_frame: str,
                                                           **kwargs) -> GetCartesianPath.Request:
        '''
        **kwargs -> cartesian planner specific arguments:
        max_step: float = 0.01, jump_threshold: float = 0.0, avoid_collisions: bool = False
        '''
        _request = GetCartesianPath.Request(
            header=Header(frame_id=planning_frame, stamp=self.get_clock().now().to_msg()),
            group_name=self.move_group_name_,
            link_name=self.end_effector_,
            waypoints=waypoints,
            max_step = kwargs.get("max_step", 0.01),
            jump_threshold = kwargs.get("jump_threshold", 0.0),
            avoid_collisions = kwargs.get("avoid_collisions", False)
        )
        return _request
    
    def _cartesian_interpolator_response_handler(self, response):
        _response_handle = {'trajectory': None, 'stop_flag': False, 'fraction': None}

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Failed to compute Cartesian path: {response.error_code.val}")
            return _response_handle

        _trajectory =  response.solution
        _fraction = response.fraction

        _response_handle['trajectory'] = _trajectory
        _response_handle['fraction'] = _fraction
        _response_handle['stop_flag'] = True
        if _fraction > 0.99:
            # _response_handle['stop_flag'] = True
            self.get_logger().info(f"===== Fraction achieved ====== {_fraction}")
        else:
            self.get_logger().error(f"===== Fraction achieved ====== {_fraction}")
        return _response_handle

    def _set_cartesian_sequence_service_client(self, service_name: str = "plan_sequence_path") -> None:
        self.spline_srv_name_ = f"{self.remapping_name_}/{service_name}" if self.remapping_name_ else service_name    
        self.spline_client_ = self.create_client(GetMotionSequence, self.spline_srv_name_)        
        if not self.spline_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: GetCartesianPath service not available -> {self.spline_client_.srv_name}.")
            exit(1)
        self.SEQUENCE_CLIENT_FLAG = True
    
    def _create_motion_sequence_request(self, waypoints: list[Pose], 
                                        planning_frame: str,
                                        **kwargs) -> MotionSequenceRequest:
        """
        Create a MotionSequenceRequest for the Pilz planner, using the provided poses and motion type.

        :param waypoints: List of Pose objects defining the Cartesian waypoints for the sequence.
        :param move_group: The name of the move group to plan for.
        :param motion_type: Type of motion ('PTP', 'LIN', 'CIRC'). Defaults to 'LIN'.
        :param blend_radius: Blending radius for transitions between sequence items. Defaults to 0.00001
        :param kwargs: Additional parameters for velocity and acceleration scaling factors.
            - max_velocity_scaling_factor: float = 0.1
            - max_acceleration_scaling_factor: float = 0.1
        :return: A populated MotionSequenceRequest.
        """
        
        sequence_request = GetMotionSequence.Request()
        

        # Loop through waypoints to create motion requests for the sequence
        for waypoint in waypoints:
            req = GetMotionPlan.Request()
            req.motion_plan_request.pipeline_id = "pilz_industrial_motion_planner"
            req.motion_plan_request.planner_id = "LIN"
            req.motion_plan_request.allowed_planning_time = 10.0
            req.motion_plan_request.group_name = self.move_group_name_
            req.motion_plan_request.max_acceleration_scaling_factor = 0.1
            req.motion_plan_request.max_velocity_scaling_factor = 0.1
            req.motion_plan_request.num_planning_attempts = 1
            
            # request.motion_plan_request.start_state = start_state

            req.motion_plan_request.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header = Header(frame_id = planning_frame),
                        link_name = self.end_effector_,
                        constraint_region = BoundingVolume(
                            primitives = [SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses = [Pose(position=waypoint.position)],
                        ),
                        weight = 1.0,
                    )
                ],
                orientation_constraints = [
                    OrientationConstraint(
                        header = Header(frame_id = planning_frame),
                        link_name = self.end_effector_,
                        orientation = waypoint.orientation,
                        absolute_x_axis_tolerance = 0.001,
                        absolute_y_axis_tolerance = 0.001,
                        absolute_z_axis_tolerance = 0.001,
                        weight = 1.0,
                    )
                ],
            )
        )
            # Add the motion request to the sequence with the specified blending radius
            sequence_item = MotionSequenceItem(
                req=req.motion_plan_request,
                blend_radius = 0.00001
            )

            sequence_request.request.items.append(sequence_item)

        if sequence_request.request.items:
            sequence_request.request.items[-1].blend_radius = 0.0
        
        return sequence_request

    def _motion_sequence_response_handler(self, response):
        """
        Handle the response from a MotionSequenceRequest.

        :param response: The response from the motion planning service.
        :return: A dictionary containing the trajectory, fraction of the path planned, and stop flag.
        """
        response_handle = {'trajectory': None, 'stop_flag': False, 'fraction': None}

        if response.response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Failed to compute motion sequence: {response.response.error_code.val}")
            return response_handle

        trajectory = response.response.planned_trajectories
        # fraction = response.fraction

        response_handle['trajectory'] = trajectory
        # response_handle['fraction'] = fraction
        response_handle['stop_flag'] = True

        return response_handle


    def _set_cartesian_sequence_action_client(self, action_name: str = "sequence_move_group") -> None:
        self.spline_action_name_ = f"{self.remapping_name_}/{action_name}" if self.remapping_name_ else action_name

        self.spline_client_ = ActionClient(self, MoveGroupSequence, self.spline_action_name_)

        if not self.spline_client_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: MoveGroupSequence action not available -> {self.spline_client_._action_name}")
            exit(1)
        self.SEQUENCE_CLIENT_FLAG = True

    def _create_motion_sequence_goal_request(self, waypoints: list[Pose], 
                                        planning_frame: str,
                                        **kwargs) -> MotionSequenceRequest:
        """
        Create a MotionSequenceRequest for the Pilz planner, using the provided poses and motion type.

        :param waypoints: List of Pose objects defining the Cartesian waypoints for the sequence.
        :param move_group: The name of the move group to plan for.
        :param motion_type: Type of motion ('PTP', 'LIN', 'CIRC'). Defaults to 'LIN'.
        :param blend_radius: Blending radius for transitions between sequence items. Defaults to 0.00001
        :param kwargs: Additional parameters for velocity and acceleration scaling factors.
            - max_velocity_scaling_factor: float = 0.1
            - max_acceleration_scaling_factor: float = 0.1
        :return: A populated MotionSequenceRequest.
        """
        
        sequence_goal_request = MoveGroupSequence.Goal() 
        
        
        # Loop through waypoints to create motion requests for the sequence
        for waypoint in waypoints:
            _waypoint_constraint = rosm._pose_to_constraints(target_pose=waypoint, frame_id=planning_frame, link_name=self.end_effector_)
            _waypoint_req = self._create_motion_plan_request(goal_constraints=[_waypoint_constraint], 
                                                             pipeline_id="pilz_industrial_motion_planner", planner_id="LIN",
                                                             allowed_planning_time=kwargs.get("allowed_planning_time", 10.0),
                                                             max_velocity_scaling_factor=kwargs.get("max_velocity_scaling_factor", 0.1),
                                                             max_acceleration_scaling_factor=kwargs.get("max_acceleration_scaling_factor", 0.1),
                                                             num_planning_attempts=kwargs.get("num_planning_attempts", 1000)
                                                        )


            # Add the motion request to the sequence with the specified blending radius
            sequence_item = MotionSequenceItem(
                req=_waypoint_req.motion_plan_request, #_waypoint_req, #
                blend_radius = 0.00001
            )

            sequence_goal_request.request.items.append(sequence_item)
  

        if sequence_goal_request.request.items:
            sequence_goal_request.request.items[-1].blend_radius = 0.0
        
        return sequence_goal_request
    
    def _motion_sequence_action_response_handler(self, response):
        """
        Handle the response from a MotionSequenceRequest.

        :param response: The response from the motion planning service.
        :return: A dictionary containing the trajectory, fraction of the path planned, and stop flag.
        """
        response_handle = {'trajectory': None, 'stop_flag': False, 'fraction': None}
        
        if response.accepted != 1:
            self.get_logger().error(f"Failed to compute motion sequence: action status {response.status}")
            return response_handle

        trajectory = response.get_result_async()
        rclpy.spin_until_future_complete(self, trajectory)
        # fraction = response.fraction

        response_handle['trajectory'] = trajectory.result().result
        # response_handle['fraction'] = fraction
        response_handle['stop_flag'] = True

        return response_handle


    #================== End of Moveit Planning ========================
    ##### create real execution client
    def modify_joint_state_for_moveit(self, joint_state):
        return rsmod.modify_joint_state(joint_state, self.prefix_, frame_id="world", add_prefix=True)

    def modify_trajectory_for_moveit(self, trajectory):
        return rsmod.modify_trajectory(trajectory, self.prefix_, frame_id="world", add_prefix=True)
