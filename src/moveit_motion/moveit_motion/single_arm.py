import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from ros_submodules.MoveitInterface import MoveitInterface

def main():
    rclpy.init()
    # client_blue = MoveitInterface(node_name="client_blue",     
    #                               move_group_name="kuka_blue", # arm # kuka_blue   #-> required for motion planning
    #                               remapping_name="",           # lbr # ""          #-> required for service and action remapping
    #                               prefix="kuka_blue",          # ""  # kuka_blue   #-> required for filtering joint states and links
    #                              )
    
    client_blue = MoveitInterface(node_name="client_blue",     
                                  move_group_name="arm", 
                                  remapping_name="lbr", 
                                  prefix="")
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
    
    cjs_blue = client_blue.get_current_joint_state()
    print(cjs_blue)
    cjs_blue_pose = client_blue.get_current_robot_pose()
    print(cjs_blue_pose)
    for pose in poses:
        tjs_blue = client_blue.get_best_ik(target_pose=pose, current_joint_state=cjs_blue, attempts=300)
        
        print(tjs_blue)
        plan_blue = client_blue.get_joint_traj(target_joint_state=tjs_blue, 
                                                  start_joint_state=cjs_blue,
                                                  planner_type="ompl")
        
        cjs_blue = tjs_blue
    
    # combined_trajectory = client_dual.combine_trajectories(dual_spline_trajectory)

    # client_dual.execute_joint_traj(combined_trajectory)
    

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
