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
from concurrent.futures import ThreadPoolExecutor, as_completed


def main(_file_name):
    rclpy.init()
    kg =None; kb = None
    
    kg =  MoveitInterface(node_name=f"client_kuka_green",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_green",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )


    kb = MoveitInterface(node_name=f"client_kuka_blue",     
                                  move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    
    # kg_cjs = kg.get_current_joint_state(); # print("kg_cjs: ", kg_cjs.position)


    path = f'no-sync/edge_3/{_file_name}'
    data_loader = cfp.DataParser.from_quat_file(file_path = path, target_fps= 60, filter=False, window_size=30, polyorder=3)
    #while not input is to quit
    FRACTION_TO_RUN = 1.0
    SLOWNESS_FACTOR = 5.0

    
    _data_chisel = data_loader.get_rigid_TxyzQwxyz()['chisel']
    _data_gripper = data_loader.get_rigid_TxyzQwxyz()['gripper']
    
    # define int(FRACTION_TO_RUN*len(_data_chisel)
    index_ = int(FRACTION_TO_RUN*len(_data_chisel))

    _data_points_chisel = _data_chisel[:index_]
    _data_points_gripper = _data_gripper[:index_]
    _data_times = data_loader.get_time()[:index_]

    print(f"Number of data points for chisel: {len(_data_points_chisel)}")
    print(f"Number of data points for gripper: {len(_data_points_gripper)}")
    print(f"Number of data points for time: {len(_data_times)}")

    _data_points_chisel = np.apply_along_axis(rm.robodk_2_ros, 1, _data_points_chisel)
    _pose_waypoints_chisel = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_chisel)
    _pose_waypoints_chisel = _pose_waypoints_chisel.tolist()

    _data_points_gripper = np.apply_along_axis(rm.robodk_2_ros, 1, _data_points_gripper)
    _pose_waypoints_gripper = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_gripper)
    _pose_waypoints_gripper = _pose_waypoints_gripper.tolist()

    # Parallelize the planning phase
    # Run kg and kb planning sequentially to avoid the spin issue
    if kg:
        kg_plan_handle = kg.get_cartesian_spline_plan(
            waypoints=_pose_waypoints_chisel, planning_frame='world',
            _planner_type="cartesian_interpolator", max_step=0.01,
            jump_threshold=0.0, avoid_collisions=False, attempts=1
        )

    # if kb:
    #     kb_plan_handle = kb.get_cartesian_spline_plan(
    #         waypoints=_pose_waypoints_gripper, planning_frame='world',
    #         _planner_type="cartesian_interpolator", max_step=0.01,
    #         jump_threshold=0.0, avoid_collisions=False, attempts=1
    #     )

    print(f"Fraction of path executed (kg): {kg_plan_handle['fraction']}\nStop flag (kg): {kg_plan_handle['stop_flag']}")
    # print(f"Fraction of path executed (kb): {kb_plan_handle['fraction']}\nStop flag (kb): {kb_plan_handle['stop_flag']}")


    # Proceed with trajectory timing correction
    if kg:
        ADD_TIMES_FLAG_CHISEL = len(_data_times) == len(_data_points_chisel)
        if len(_data_times) > 0 and not ADD_TIMES_FLAG_CHISEL:
            kg.get_logger().error("Invalid time_stamps provided")
            return None

        if ADD_TIMES_FLAG_CHISEL and kg_plan_handle['stop_flag']:
            _completed_time_steps = int(len(_data_times) * kg_plan_handle['fraction'])
            kg_plan_handle['trajectory'] = rosm.interpolate_trajectory_timestamps(kg_plan_handle['trajectory'], _data_times[:_completed_time_steps], scaling_factor=SLOWNESS_FACTOR)

    # if kb:
    #     ADD_TIMES_FLAG_GRIPPER = len(_data_times) == len(_data_points_gripper)
    #     if len(_data_times) > 0 and not ADD_TIMES_FLAG_GRIPPER:
    #         kb.get_logger().error("Invalid time_stamps provided")
    #         return None

    #     if ADD_TIMES_FLAG_GRIPPER and kb_plan_handle['stop_flag']:
    #         _completed_time_steps = int(len(_data_times) * kb_plan_handle['fraction'])
    #         kb_plan_handle['trajectory'] = rosm.interpolate_trajectory_timestamps(kb_plan_handle['trajectory'], _data_times[:_completed_time_steps], scaling_factor=SLOWNESS_FACTOR)


    # Execute both trajectories simultaneously
    EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()

    if EXECUTE_FLAG == 'y':
        if kg:
            kg.execute_joint_traj(kg_plan_handle['trajectory'])
        
        # if kb:
        #     kb.execute_joint_traj(kb_plan_handle['trajectory'])
        # with ThreadPoolExecutor() as executor:
        #     future_exec_kg = executor.submit(kg.execute_joint_traj, kg_plan_handle['trajectory'])
        #     future_exec_kb = executor.submit(kb.execute_joint_traj, kb_plan_handle['trajectory'])
            
        #     # Ensure both have started simultaneously
        #     future_exec_kg.result()
        #     future_exec_kb.result()

    else:
        print("Trajectory not executed")

    # LOOP_BREAK_FLAG = input("\n--------> ALERT !!! Continue? (y/n): ").strip().lower()
    # if not LOOP_BREAK_FLAG == 'y':
    rclpy.shutdown()

if __name__ == '__main__':
    import sys

    _file_name = sys.argv[2] if len(sys.argv) > 2 else "ft_010.csv"

    main(_file_name)


