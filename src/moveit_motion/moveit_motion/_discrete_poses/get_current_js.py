import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface
import copy
import moveit_motion.ros_submodules.ros_math as rosm
import numpy as np
import moveit_motion.diffusion_policy_cam.submodules.cleaned_file_parser as cfp
import moveit_motion.diffusion_policy_cam.submodules.robomath_addon as rma
import moveit_motion.diffusion_policy_cam.submodules.robomath as rm
import time
import csv
from math import pi
from moveit_motion.ros_submodules.RobotInterface import RobotInterface
import moveit_motion.ros_submodules.RS_submodules as rsmod
from concurrent.futures import ThreadPoolExecutor, as_completed


def main():
    rclpy.init()
    kg =None; kb = None    
    # kg = MoveitInterface(node_name=f"client_real_kuka_green",     
    #                               move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
    #                               remapping_name="kuka_green",           # lbr # ""          #-> required for service and action remapping
    #                               prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                              )


    kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
                                    move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                    remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                    prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                    )


    if kb:
        kb_cjs = kb.get_current_joint_state()
        print("kb_cjs: ", kb_cjs)

        for name, pos in zip(kb_cjs.name, kb_cjs.position):
            print(f"{name}: {pos}")

    if kg:
        kg_cjs = kg.get_current_joint_state()
        print("kg_cjs: ", kg_cjs)

        for name, pos in zip(kg_cjs.name, kg_cjs.position):
            print(f"{name}: {pos}")


    rclpy.shutdown()


if __name__ == '__main__':
    main()