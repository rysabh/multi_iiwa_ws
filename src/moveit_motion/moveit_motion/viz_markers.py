import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState

from ros_submodules.MoveitInterface import MoveitInterface

from diffusion_policy_cam.submodules import cleaned_file_parser as cfp, robomath_addon as rma, robomath as rm

from ros_submodules.rviz_link import RvizLink

import ros_submodules.ros_math as ros_m
# from diffusion_policy_cam.submodules import cleaned_file_parser as cfp, robomath_addon as rma, robomath as rm

import pandas as pd
import numpy as np



def main():
    rclpy.init()

    file_path = "src/moveit_motion/moveit_motion/diffusion_policy_cam/diffusion_pipline/data_chisel_task/cleaned_traj/cap_010_cleaned.csv"    

    data = cfp.DataParser.from_quat_file(file_path = file_path, target_fps= 120.0, filter=True, window_size=15, polyorder=3)

    RL = RvizLink()
    RL.pub_robot(parent_frame='world', base_link='kuka_green_link_0', translation=(0.5, 0.5, 0.009), rotation=(0.0, 0.0, 0.0, 1.0))
 
    RL.create_marker_publisher()
    print()
    # RL.pub_marker(namespace='m', frame_id='world', translation=[0,0,0], rotation=[0,0,0,1], scale=(0.3, 0.3, 0.3), color=(0.0, 1.0, 0.0, 1.0))
    # print()
    data_chisel = data.get_rigid_TxyzQwxyz()["chisel"]
    # data_times = data.get_time()

    data_chisel = np.apply_along_axis(ros_m.robodk_2_ros, 1, data_chisel)

    RL.pub_markers(markers=data_chisel, frame_id='world')

    RL.start_simulation(interval=1/10)
    rclpy.spin(RL)

if __name__ == "__main__":
    main()