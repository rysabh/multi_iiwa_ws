import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from ros_submodules.MoveitInterface import MoveitInterface
import ros_submodules.ros_math as rosm

import numpy as np
def main():
    rclpy.init()
    client = MoveitInterface(node_name="client",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="",           # lbr # ""          #-> required for service and action remapping
                                  prefix="kuka_green",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    
    # client = MoveitInterface(node_name="client",     
    #                               move_group_name="arm", 
    #                               remapping_name="lbr", 
    #                               prefix="")
    # poses = [
    #     Pose(
    #             position=Point(x=0.6, y=0.0, z=0.6),
    #             orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
    #         ),
    #     Pose(
    #             position=Point(x=0.5, y=0.1, z=0.4),
    #             orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
    #         ),
    #     Pose(
    #             position=Point(x=0.0, y=0.0, z=1.266),
    #             orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    #         )
    # ]
    
    start_joint_state = rosm.joint_list_2_state(joint_positions=[0.0, 0.53, -0.0, -0.94, 0.0, 1.67, -0.0], joint_names=client.get_current_joint_state().name)
    
    current_pose = client.get_current_robot_pose()
    ROTi = current_pose.orientation; ROTi = [ROTi.x, ROTi.y, ROTi.z, ROTi.w]
    # POSi = current_pose.position; POSi = [POSi.x, POSi.y, POSi.z]
    
    waypoints = [
        [1.0331, 1.0706, 1.1766]+ ROTi,
        [1.0837, 1.1974, 1.1337]+ ROTi,
        [1.1651, 1.2861, 1.0486]+ ROTi,
        [1.2446, 1.1808, 1.0236]+ ROTi,
        [1.2616, 1.0293, 1.0236]+ ROTi,
    ]
    
    #convert waypoint to poses using rosm.TxyzQwxyz_2_Pose
    poses = []
    for waypoint in waypoints:
        pose = rosm.TxyzQwxyz_2_Pose(waypoint)
        poses.append(pose)

    plan_solution, fraction_completed = client.get_cartesian_path(waypoints=poses, attempts=300)
    
    print(fraction_completed)

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
