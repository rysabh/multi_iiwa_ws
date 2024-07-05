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


from submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states, filter_joints_for_move_group_name

from MoveitActionClient import MoveitActionClient

def main():
    rclpy.init()

    client_blue = MoveitActionClient(
        node_name="client_blue",
        move_group_name="kuka_blue",
        sim=True
    )

    client_green = MoveitActionClient(
        node_name="client_green",
        move_group_name="kuka_green",
        sim=True
    )

    client_dual = MoveitActionClient(
        node_name="client_dual",
        move_group_name="dual_arm",
        sim=True
    )

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

        planned_joint_trajectory = client_dual.get_joint_space_motion_plan(target_joint_state=dual_inverse_list)

        if planned_joint_trajectory is None:
            print("no planned trajectory")
            return
        
        dual_spline_trajectory.append(planned_joint_trajectory)

        client_dual.execute_joint_traj_sim(planned_joint_trajectory)
    

    rclpy.shutdown()
    
    '''
    green_traj_points = []
    blue_traj_points = []
    green_spline_trajectory = []
    blue_spline_trajectory = []

    for point in planned_joint_trajectory.points:
        green_waypoint = JointTrajectoryPoint()
        blue_waypoint = JointTrajectoryPoint()
        robot_dof = int(len(point.positions) / 2)
        green_waypoint.positions = point.positions[robot_dof:]
        blue_waypoint.positions = point.positions[:robot_dof]
        green_waypoint.accelerations = point.accelerations[robot_dof:]
        blue_waypoint.accelerations = point.accelerations[:robot_dof]
        green_waypoint.velocities = point.velocities[robot_dof:]
        blue_waypoint.velocities = point.velocities[:robot_dof]
        green_waypoint.effort = point.effort
        blue_waypoint.effort = point.effort
        green_waypoint.time_from_start = point.time_from_start
        blue_waypoint.time_from_start = point.time_from_start
        green_traj_points.append(green_waypoint)
        blue_traj_points.append(blue_waypoint)

    if not total_trajectory.points:
        last_time = 0.0
    else:
        last_point = total_trajectory.points[-1]
        last_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9

    for point in trajectory.points:
        new_point = JointTrajectoryPoint()
        new_point.positions = point.positions
        new_point.velocities = point.velocities
        new_point.accelerations = point.accelerations
        new_point.effort = point.effort
        new_point.time_from_start.sec = point.time_from_start.sec + int(last_time)
        new_point.time_from_start.nanosec = point.time_from_start.nanosec + int((last_time - int(last_time)) * 1e9)
        self.total_trajectory.points.append(new_point)

    current_state_dual_inverse_list = dual_inverse_list

    green_joint_traj = JointTrajectory()
    green_joint_traj.header.stamp.sec = 0
    green_joint_traj.header.stamp.nanosec = 0
    green_joint_traj.header.frame_id = ''
    green_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
    green_joint_traj.points = green_traj_points

    blue_joint_traj = JointTrajectory()
    blue_joint_traj.header.stamp.sec = 0
    blue_joint_traj.header.stamp.nanosec = 0
    blue_joint_traj.header.frame_id = ''
    blue_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
    blue_joint_traj.points = blue_traj_points

    save_trajectory(blue_joint_traj, "kuka_blue.dump")
    save_trajectory(green_joint_traj, "kuka_green.dump")
    save_trajectory_to_csv(blue_joint_traj, "kuka_blue.csv")
    save_trajectory_to_csv(green_joint_traj, "kuka_green.csv")
    '''


if __name__ == '__main__':
    main()
