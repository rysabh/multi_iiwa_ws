#NOT WORKING
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from moveit_motion.ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface

from moveit_motion.ros_submodules.ros_math import joint_points_2_trajectory
import numpy as np



def main():
    rclpy.init()
    client = MoveitInterface(node_name="client",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="",           # lbr # ""          #-> required for service and action remapping
                                  prefix="kuka_green",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    

    cjs = client.get_current_joint_state()

    csv_path = os.path.join(os.path.dirname(__file__), "joint_states.csv")

    joint_traj = joint_points_2_trajectory(csv_path=csv_path, joint_names=cjs.name)

    client.execute_joint_traj(joint_traj)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
