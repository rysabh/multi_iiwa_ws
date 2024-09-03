#NOT WORKING
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface

import moveit_motion.ros_submodules.ros_math as rosm
import numpy as np
import moveit_motion.diffusion_policy_cam.submodules.cleaned_file_parser as cfp
import moveit_motion.ros_submodules.ros_math as rm
import csv
from math import pi
from moveit_motion.ros_submodules.RobotInterface import RobotInterface
import moveit_motion.ros_submodules.RS_submodules as rsmod


def main(_robot_name, _file_name):

    rclpy.init()
    
    kgr = RobotInterface(node_name=f"client_{_robot_name}_real",     
                                  move_group_name=_robot_name, # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name=_robot_name,           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    
    kg =  MoveitInterface(node_name=f"client_{_robot_name}",     
                                  move_group_name=_robot_name, # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="",           # lbr # ""          #-> required for service and action remapping
                                  prefix=_robot_name,          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )

    # kg =  MoveitInterface(node_name=f"client_{_robot_name}",     
    #                             move_group_name="arm", # arm # kuka_g/b..   #-> required for motion planning
    #                             remapping_name="kuka_green",           # lbr # ""          #-> required for service and action remapping
    #                             prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                             )

    kg_cjs = kg.get_current_joint_state()

    kgr_cjs = kgr.get_current_joint_state()

    # print("kg_cjs: ", kg_cjs.position)
    # print("kgr_cjs: ", kgr_cjs.position)


    kg_njs = kg.modify_joint_state_for_moveit(kgr_cjs)
    # print("kg_njs: ", kg_njs)

    m1 =  kg.get_joint_ptp_plan(start_joint_state=kg_cjs, target_joint_state=kg_njs)['trajectory']
    kg.execute_joint_traj(m1)



    kgr_cgs_moveit = kg.modify_joint_state_for_moveit(kgr_cjs)

    

    test_js = rosm.joint_list_2_state([-1.03, -0.262, -0.384, 0.925, 0.0, -0.663, 1.571], kg_cjs.name)
    # test_js = kg.get_current_joint_state()
    # print(rosm.joint_state_2_list(test_js, verbose=True))

    m2 = kg.get_joint_ptp_plan(start_joint_state=kgr_cgs_moveit, target_joint_state=test_js)['trajectory']
    m2r = kgr.modify_trajectory_for_robot(m2)
    print("executing joint trajectory")
    # kg.execute_joint_traj(m2)
    kgr.execute_joint_traj(m2r)
    #take sim robot to the same position as the real robot

 
    
    
    rclpy.shutdown()


if __name__ == '__main__':
    import sys

    _robot_name = sys.argv[1] if len(sys.argv) > 1 else "kuka_green"
    _file_name = sys.argv[2] if len(sys.argv) > 2 else "ft_010.csv"

    main(_robot_name, _file_name)


