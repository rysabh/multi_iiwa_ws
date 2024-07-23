import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
# from typing import Optional  

# from moveit_msgs.action import MoveGroup, ExecuteTrajectory
# from moveit_msgs.msg import (
#     RobotState,
#     RobotTrajectory,
#     MoveItErrorCodes,
#     Constraints,
#     JointConstraint,
# )
# from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetPositionFK

from sensor_msgs.msg import JointState

# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# from submodules.wait_for_message import wait_for_message


from ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from ros_submodules.MoveitInterface import MoveitInterface

def main():
    rclpy.init()

    client_blue = MoveitInterface(node_name="client_blue", move_group_name="kuka_blue")

    client_green = MoveitInterface(node_name="client_green", move_group_name="kuka_green")

    client_dual = MoveitInterface(node_name="client_dual", move_group_name="dual_arm")

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

    # kuka_blue_current_joint_state = client_blue.get_robot_current_joint_state()
    # kuka_blue_current_joint_state = client_blue.modify_joint_state_for_sim_robot(kuka_blue_current_joint_state)

    # kuka_green_current_joint_state = client_green.get_robot_current_joint_state()
    # kuka_green_current_joint_state = client_green.modify_joint_state_for_sim_robot(kuka_green_current_joint_state)


    dual_spline_trajectory = []
    
    cjs_dual = client_dual.get_current_joint_state() #current joint state for dual arm
    cjs_green = client_green.get_current_joint_state()
    cjs_blue = client_blue.get_current_joint_state()

    for pose in poses:
        tjs_blue = client_blue.get_best_ik(target_pose=pose, current_joint_state=cjs_blue)
        tjs_green = client_green.get_best_ik(target_pose=pose, current_joint_state=cjs_green)
        
        # if tjs_blue is None or tjs_green is None:
        #     print("IK not Found for current pose : exiting the main function")
        #     return

        tjs_dual = JointState() #target joint state for dual arm
        tjs_dual.effort = tjs_blue.effort + tjs_green.effort
        tjs_dual.header = tjs_blue.header
        tjs_dual.name = tjs_blue.name + tjs_green.name
        tjs_dual.position = tjs_blue.position + tjs_green.position
        tjs_dual.velocity = tjs_blue.velocity + tjs_blue.velocity

        # planned_joint_trajectory = client_dual.get_joint_traj(target_joint_state=tjs_dual, 
        #                                                       start_joint_state=cjs_dual,
        #                                                       planner_type="pilz")        
        # if planned_joint_trajectory is None:
        #     print("no planned trajectory")
        #     return
        # dual_spline_trajectory.append(planned_joint_trajectory)
        
        plan_green = client_green.get_joint_traj(target_joint_state=tjs_green, 
                                                  start_joint_state=cjs_green,
                                                  planner_type="pilz")

        
        
        
        cjs_blue = tjs_blue
        cjs_green = tjs_green
        cjs_dual = tjs_dual
    
    # combined_trajectory = client_dual.combine_trajectories(dual_spline_trajectory)

    # client_dual.execute_joint_traj(combined_trajectory)
    

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
