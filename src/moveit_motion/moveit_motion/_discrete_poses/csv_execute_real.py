#NOT WORKING
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from moveit_motion.ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface

import moveit_motion.ros_submodules.ros_math as rosm
import numpy as np
import moveit_motion.diffusion_policy_cam.submodules.cleaned_file_parser as cfp
import moveit_motion.ros_submodules.ros_math as rm
import csv
from math import pi
from moveit_motion.ros_submodules.RobotInterface import RobotInterface


def main(_robot_name, _file_name):

    rclpy.init()
    
    client = RobotInterface(node_name=f"client_{_robot_name}",     
                                  move_group_name=_robot_name, # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name=_robot_name,           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    

    cjs = client.get_current_joint_state()
    
    print(cjs)

    rclpy.shutdown()


if __name__ == '__main__':
    import sys

    _robot_name = sys.argv[1] if len(sys.argv) > 1 else "kuka_green"
    _file_name = sys.argv[2] if len(sys.argv) > 2 else "ft_010.csv"

    main(_robot_name, _file_name)


