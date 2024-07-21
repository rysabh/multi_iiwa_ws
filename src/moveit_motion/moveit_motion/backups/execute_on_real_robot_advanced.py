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


from ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states, filter_joints_for_move_group_name

from MoveitInterface import MoveitInterface

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
            )
    ]

    # kuka_blue_current_joint_state = client_blue.get_robot_current_joint_state()
    # kuka_blue_current_joint_state = client_blue.modify_joint_state_for_sim_robot(kuka_blue_current_joint_state)

    # kuka_green_current_joint_state = client_green.get_robot_current_joint_state()
    # kuka_green_current_joint_state = client_green.modify_joint_state_for_sim_robot(kuka_green_current_joint_state)


    dual_spline_trajectory = []

    for pose in poses:
        inverse_blue = client_blue.get_best_ik(pose)
        inverse_green = client_green.get_best_ik(pose)
        
        # if inverse_blue is None or inverse_green is None:
        #     print("IK not Found for current pose : exiting the main function")
        #     return

        dual_inverse_list = JointState()
        dual_inverse_list.effort = inverse_blue.effort + inverse_green.effort
        dual_inverse_list.header = inverse_blue.header
        dual_inverse_list.name = inverse_blue.name + inverse_green.name
        dual_inverse_list.position = inverse_blue.position + inverse_green.position
        dual_inverse_list.velocity = inverse_blue.velocity + inverse_blue.velocity

        planned_joint_trajectory = client_dual.get_joint_traj(target_joint_state=dual_inverse_list)

        if planned_joint_trajectory is None:
            print("no planned trajectory")
            return
        
        dual_spline_trajectory.append(planned_joint_trajectory)

        client_dual.execute_joint_traj(planned_joint_trajectory)
    

    rclpy.shutdown()
    



if __name__ == '__main__':
    main()
