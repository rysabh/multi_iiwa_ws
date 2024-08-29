#WORKS

import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from moveit_motion.ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface

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
    poses = [
        Pose(
                position=Point(x=0.6, y=0.0, z=0.6),
                orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
            ),
        Pose(
                position=Point(x=0.5, y=0.1, z=0.4),
                orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
            ),
        Pose(
                position=Point(x=0.0, y=0.0, z=1.266),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
    ]

    dual_spline_trajectory = []
    
    cjs = client.get_current_joint_state()
    # print(cjs)
    # cjs_pose = client.get_current_robot_pose()
    # print(cjs_pose)
    for pose in poses:
        tjs = client.get_best_ik(target_pose=pose, current_joint_state=cjs, attempts=300)
        
        # print(tjs)
        plan = client.get_joint_plan(target_joint_state=tjs, 
                                                  start_joint_state=cjs,
                                                  planner_type="ompl")
        
        cjs = tjs
    
    # combined_trajectory = client_dual.combine_trajectories(dual_spline_trajectory)

    # client_dual.execute_joint_traj(combined_trajectory)
    

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
