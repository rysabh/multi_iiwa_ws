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
def main():
    rclpy.init()
    client = MoveitInterface(node_name="client",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="",           # lbr # ""          #-> required for service and action remapping
                                  prefix="kuka_green",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    

    cjs = client.get_current_joint_state()

    # path = 'no-sync/ik_results_1_degrees.csv'
    path = 'no-sync/2024-08-28_15-33-26.csv'
    with open(path, 'r') as f:
        reader = csv.reader(f)
        _header = next(reader)
        data = []
        # convert degrees to radians
        for row in reader:
            data.append([float(i)*pi/180 for i in row])
        

    fjs = rosm.joint_list_2_state(joint_positions = data[0], joint_names = cjs.name)

    # initial_plan = client.get_joint_ptp_plan(start_joint_state= cjs, target_joint_state=fjs, planner_type= "ompl", attempts= 100)
    
    # client.execute_joint_traj(initial_plan)

    joint_robot_traj = rosm.joint_points_2_trajectory(points = data,
                                                times= None,
                                                header_frame_id= f"{client.move_group_name_}_link_0",
                                                joint_names= cjs.name,
                                                sampling_rate= 5.0)

    client.execute_joint_traj(joint_robot_traj)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
