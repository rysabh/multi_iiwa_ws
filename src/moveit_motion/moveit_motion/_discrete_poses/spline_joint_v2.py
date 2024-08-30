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
    
    
    joint_waypoints = [
        [3.0, 4.0, 3.0, -2.0, -2.0, 3.0, -1.0],
        [11.0, 10.0, 7.0, -5.0, -7.0, 9.0, -5.0],
        [16.0, 14.0, 19.0, -12.0, -17.0, 14.0, -16.0],
        [21.0, 20.0, 33.0, -19.0, -28.0, 20.0, -31.0], # add more waypoints here with the same format but bigger offset
        [26.0, 26.0, 47.0, -26.0, -39.0, 26.0, -46.0],
        [31.0, 32.0, 61.0, -33.0, -50.0, 32.0, -61.0],
    ]
    # convert angles to radians (from degrees) compact way using np array
    joint_waypoints = np.radians(joint_waypoints)
    
    # joint_waypoints =  [
    #     [ 0. , 0.53 , -0.  , -0.94 , 0.  ,  1.67 ,-0.  ],
    #     [ 0.2  , 0.4 , -0.  , -1.65 , 0.  ,  1.1  , 0.2 ],
    #     [0.,0.,0.,0.,0.,0.,0.]
    #  ]
    

    cjs = client.get_current_joint_state()

    waypoints = []
    
    for waypoint in joint_waypoints:
        angles = list(waypoint)
        joint_state = rosm.joint_list_2_state(joint_positions=angles, joint_names=cjs.name)
        waypoints.append(joint_state)
    
    # print(waypoints)
    # plan = client.get_joint_spline(waypoints=waypoints, planner_type="ompl")
    plan = client.get_joint_spline(waypoints, 
                                 planner_type="ompl", attempts=300,
                                 max_acceleration_scaling_factor = 0.6,
                                 max_velocity_scaling_factor = 0.6,
                                 )
    

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
