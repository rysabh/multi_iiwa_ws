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
from lbr_fri_idl.msg import LBRState
from typing import Optional, Union
from std_msgs.msg import Header

from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
)

from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory

from moveit_motion.ros_submodules.wait_for_message import wait_for_message

import numpy as np

import moveit_motion.ros_submodules.RS_submodules as rsmod
import moveit_motion.ros_submodules.ros_math as rosm


import os
from control_msgs.action import FollowJointTrajectory

class RobotInterface(Node):
    '''
    RobotInterface(
        node_name="real_client_blue",      # lbr / kuka_blue -> required for FK service
        move_group_name="kuka_blue",  # arm / kuka_blue -> required for motion planning
        remapping_name="kuka_blue",            # lbr / ""        -> required for service and action remapping
        prefix="kuka_blue",           # ""  / kuka_blue -> required for filtering joint states and links
    ) 
    '''

    THRESHOLD_2_MOVE = 0.0005 # 0.0005
    EXECUTE_ACTION_CLIENT_FLAG = False

    def __init__(self, node_name, move_group_name="arm", remapping_name="lbr", prefix=""):
        super().__init__(node_name)
        self.node_name_ = node_name
        self.timeout_sec_ = 10.0
        self.move_group_name_ = move_group_name
        self.remapping_name_ = remapping_name
        self.prefix_ = prefix
        self.joint_states_topic_ = f"{remapping_name}/joint_states" if remapping_name else "joint_states"



    def _set_execute_action_client(self, action_name: str = "follow_joint_trajectory") -> None:
        # self.execute_action_name_ = f"{self.remapping_name_}_controller/{action_name}" if self.remapping_name_ else action_name
        self.execute_action_name_ = f"/{self.move_group_name_}/joint_trajectory_controller/follow_joint_trajectory"
        self.execute_client_ = ActionClient(self, FollowJointTrajectory, self.execute_action_name_)

        if not self.execute_client_.wait_for_server(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: ExecuteActionClient-REAL action not available -> {self.execute_client_._action_name}")
            exit(1)
        self.EXECUTE_ACTION_CLIENT_FLAG = True


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

    #===================================================================
    ###################### MOVEIT EXECUTION ############################
    #===================================================================
    def execute_joint_traj(self, trajectory: RobotTrajectory) -> None:
        if self.EXECUTE_ACTION_CLIENT_FLAG == False:
            self._set_execute_action_client()
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        goal_future = self.execute_client_.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        
        goal_handle = goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Failed to execute trajectory, goal rejected by action server: {goal_handle.get_result()}")
            return None
        
        self.get_logger().info("Trajectory accepted, moving the REAL robot...")
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        if not result_future.done():
            self.get_logger().info("!!! Trajectory REAL NOT executed")
            raise RuntimeError
        
        self.get_logger().info("Trajectory REAL executed")
        return
    #================== End of Moveit Execution ========================


    def modify_trajectory_for_robot(self, trajectory):
        return rsmod.modify_trajectory(trajectory, self.remapping_name_, frame_id='', add_prefix=False)


    def modify_joint_state_for_robot(self, joint_state):
        return rsmod.modify_joint_state(joint_state, self.remapping_name_, frame_id='', add_prefix=False)
    

if __name__ == "__main__":
    rclpy.init()
    try:
        client =  RobotInterface(
            node_name="real_client_blue",      # lbr / kuka_blue -> required for FK service
            move_group_name="kuka_blue",  # arm / kuka_blue -> required for motion planning
            remapping_name="kuka_blue",            # lbr / ""        -> required for service and action remapping
            prefix="",           # ""  / kuka_blue -> required for filtering joint states and links
            ) 
           
        cjs = client.get_current_joint_state()
        print(cjs)
    finally:
        rclpy.shutdown()
