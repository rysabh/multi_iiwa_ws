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

    data = cfp.DataParser.from_quat_file(file_path = file_path, target_fps= 10.0, filter=True, window_size=15, polyorder=3)

    RL = RvizLink()

    data_chisel = data.get_rigid_TxyzQwxyz()["chisel"]
    data_times = data.get_time()

    data_chisel = np.apply_along_axis(ros_m.robodk_2_ros, 1, data_chisel)

    RL.pub_markers(markers=data_chisel, frame_id='world')


if __name__ == "__main__":
    main()