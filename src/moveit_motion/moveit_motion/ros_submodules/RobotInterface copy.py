# GNU GENERAL PUBLIC LICENSE
# Version 3, 29 June 2007
# 
# Copyright (C) 2024 Rishabh Shukla
# email: rysabh@gmail.com
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# see <https://www.gnu.org/licenses/>.
# 
# Written by Rishabh Shukla

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

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ros_submodules.wait_for_message import wait_for_message

import numpy as np

from ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states, filter_joints_for_move_group_name


class RobotInterface(Node):
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

        
        self.execute_client_ = ActionClient(self, ExecuteTrajectory, self.execute_action_name_)
        if not self.execute_client_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: ExecuteActionClient-Sim action not available -> {self.execute_client_._action_name}")
            exit(1)
        
        # Execute Client for Real Robot
        self.execute_client_real_ = None
        
    def add_real_execution_client(self):
        execute_action_name_real_ =  f"/{self.move_group_name_}/joint_trajectory_controller/follow_joint_trajectory"
        self.execute_client_real_ = ActionClient(self, FollowJointTrajectory, execute_action_name_real_)
        if not self.execute_client_real_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: ExecuteActionClient-REAL action not available -> {self.execute_client_real_._action_name}")
            exit(1)



    @filter_joints_for_move_group_name
    def get_current_joint_state(self): # use descriptors for filtering
        '''
        Joint State for the moveit move group
        '''
        _JOINT_STATE_TOPIC = "/joint_states"
        if self.sim_ == False:
            _JOINT_STATE_TOPIC = f"/{self.move_group_name_}/joint_states"

        _MSG_RECEIVED_BOOL, _current_joint_state = wait_for_message(
            JointState, self, _JOINT_STATE_TOPIC, time_to_wait=self.timeout_sec_
        )
        if not _MSG_RECEIVED_BOOL:
            self.get_logger().error("Failed to get current joint state")
            return None
        
        return _current_joint_state

    def get_robot_current_pose(self) -> Pose | None:
        '''
        Pose for the real robot
        '''
        _current_joint_state = self.get_robot_current_joint_state()
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
    
    def get_best_ik(self, target_pose: Pose, attempts: int = 100) -> JointState | None:
        current_joint_state = self.get_current_joint_state()
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
            self.get_logger().info(f'''joint state motion ->
{[round(_val, 2) for _val in current_joint_state.position]} -> {[round(_val, 2) for _val in best_joint_state.position]}
                                    ''')
        return best_joint_state


    def get_joint_space_motion_plan(self, 
                                    target_joint_state: JointState,
                                     start_joint_state: JointState | None = None, 
                                              attempts: int = 10,
                                               **kwargs
                                    ) -> JointTrajectory | None:
        '''
        kwargs = {
                    planner_type = "linear" | None,
                   }
        ''' 
        ## Create start state using start_joint_state #TODO automate sim vs real
        # if start_joint_state is None:
        #     start_joint_state = self.get_robot_current_joint_state()

        if start_joint_state is None:
            start_joint_state = self.get_current_joint_state()
        
        
        ## if start and target match exit the function
        _THRESHOLD = 0.01 # threshold from start and target for robot to move
        if MSE_joint_states(target_joint_state, start_joint_state) <= _THRESHOLD:
            self.get_logger().info("Start and End goals match. ** NOT ** moving anything and Passing Empty Trajectory")
            return JointTrajectory()
        
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
                return response.motion_plan_response.trajectory.joint_trajectory
        return None


    def execute_joint_traj_sim(self, trajectory: JointTrajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = trajectory
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

    def execute_joint_traj_real(self, trajectory: JointTrajectory):
        if not self.execute_client_real_:
            self.add_real_execution_client()

        #TODO
        #1. Modify trajectory for real robot
        #2. Add a check if starting point does not match for real robot current joint state and trajectory -> raise RuntimeError
        #3. Check difference between joint trajectory and multi_dof joint trajectory in ExecuteTrajectory.Goal().multidof_joint trajectory -> is it mor multiple robots ??
        #4. Create a spline trajectroy for multiple waypoints
        #5. add live visualization using robot_state topic and then run that in paralle to real execution or at least run sim execution in paralle to real execution
        #6. I would say, dont bother. Split the code into RobotInterface and RobotInterface classes. Then have same named functions in both for execution
        #7. Think about adding RobotName and MoveGroupname separately
        #8. Visualize yellow line path in moveit

        goal = FollowJointTrajectory.Goal()
        

        #9. TODO - This is different from old code I wrote. There it was  goal.trajectory = joint_trajectory
        goal.trajectory.joint_trajectory = trajectory
        goal_future = self.execute_client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Failed to execute trajectory")
            return None
        self.get_logger().info("Trajectory accepted, moving the robot...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not result_future.done():
            self.get_logger().info("!!! Trajectory NOT executed")
            raise RuntimeError
        
        self.get_logger().info("Trajectory executed")
        return



    @staticmethod
    def convert_joint_position_2_state(joint_positions: list, tmp) -> JointState: #TODO get rid of tmp
        _joint_state = JointState()
        _joint_state.effort = tmp.effort
        _joint_state.header = tmp.header
        _joint_state.name = tmp.name
        _joint_state.position = joint_positions
        _joint_state.velocity = tmp.velocity



## Main ###########
# #################### 
def main():
    rclpy.init()

    client_blue = RobotInterface(
        node_name="client_blue",
        move_group_name="kuka_blue"
    )

    client_green = RobotInterface(
        node_name="client_green",
        move_group_name="kuka_green"
    )

    client_dual = RobotInterface(
        node_name="client_dual",
        move_group_name="dual_arm"
    )

    poses = [
        Pose(
                position=Point(x=0.6, y=0.0, z=0.6),
                orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
            ),
        Pose(
                position=Point(x=0.5, y=0.1, z=0.4),
                orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
            )
    ]

    dual_spline_trajectory = []

    for pose in poses:
        inverse_blue = client_blue.get_best_ik(pose)
        inverse_green = client_green.get_best_ik(pose)
        
        # if inverse_blue is None or inverse_green is None:
        #     print("IK not Found for current pose : exiting the main function")
        #     return

        dual_inverse_list = JointState()
        dual_inverse_list.effort = inverse_blue.effort + inverse_green.effort
        dual_inverse_list.header = inverse_blue.header
        dual_inverse_list.name = inverse_blue.name + inverse_green.name
        dual_inverse_list.position = inverse_blue.position + inverse_green.position
        dual_inverse_list.velocity = inverse_blue.velocity + inverse_blue.velocity

        planned_joint_trajectory = client_dual.get_joint_space_motion_plan(target_joint_state=dual_inverse_list)

        if planned_joint_trajectory is None:
            print("no planned trajectory")
            return
        
        dual_spline_trajectory.append(planned_joint_trajectory)

        client_dual.execute_joint_traj_sim(planned_joint_trajectory)
    

    rclpy.shutdown()


if __name__ == '__main__':
    main()
