#NOT WORKING
from __future__ import annotations
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface
import copy
import moveit_motion.ros_submodules.ros_math as rosm
import numpy as np
import moveit_motion.diffusion_policy_cam.submodules.cleaned_file_parser as cfp
import moveit_motion.diffusion_policy_cam.submodules.robomath_addon as rma
import moveit_motion.diffusion_policy_cam.submodules.robomath as rm
import time
import csv
from math import pi
from moveit_motion.ros_submodules.RobotInterface import RobotInterface
import moveit_motion.ros_submodules.RS_submodules as rsmod
from concurrent.futures import ThreadPoolExecutor, as_completed



import csv
from pathlib import Path
import numpy as np

_HEADER = ["X", "Y", "Z", "x", "y", "z", "w"]
def read_waypoints_csv(path):
    """
    Read a CSV file written by :func:`export_waypoints_csv`.

    Returns
    -------
    np.ndarray, shape(N_pts, 7)
    """
    path = Path(path)
    with path.open(newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        if [h.strip() for h in header] != _HEADER:
            raise ValueError(
                f"Unexpected header {header} – expected {_HEADER!r}"
            )
        data = [[float(x) for x in row] for row in reader]

    return np.asarray(data, dtype=float)



def move_client_ptp(_client, goal_list: list, tolerance=0.0005, time_out=60):
    _cjs = _client.get_current_joint_state()
    _goal_state = rosm.joint_list_2_state(goal_list, _cjs.name)

    if rsmod.MSE_joint_states(_cjs, _goal_state) < tolerance:
        print("Already at goal")
        return
    
    _goal_plan_handle = _client.get_joint_ptp_plan(_cjs, _goal_state, max_velocity_scaling_factor=0.1)
    
    if input(f"Execute {_client.move_group_name_} plan? (y/n): ").strip().lower() == 'y':
        _client.execute_joint_traj(_goal_plan_handle['trajectory'])

        _tick = time.time()
        execution_finished = False
        print("Waiting for execution to finish...")
        while not execution_finished:
            mse_client = get_mse_planend_current(_client, _goal_plan_handle)

            if (mse_client < 0.0002):
                execution_finished = True
            
            _tock = time.time()

            if _tock - _tick > time_out: print(f"Timeout {time_out} seconds: Execution not finished"); break
            time.sleep(0.01)


def plan_client_cartesian(_client, waypoints: list, max_motion_threshold= float, max_attemps: int = 5):
    planner_type = "cartesian_interpolator"
    for _attempt in range(max_attemps):
        _cartesian_plan_handle = _client.get_cartesian_spline_plan(
            waypoints=waypoints, planning_frame='world',
            attempts=1,
            # _planner_type="cartesian_sequence_action", 
            _planner_type = planner_type,
            allowed_planning_time=10.0, max_velocity_scaling_factor=0.01,
            max_acceleration_scaling_factor=0.01, num_planning_attempts=100
        )

        if planner_type == "sequence_move_group":
            return
        
        _start_traj_point = _cartesian_plan_handle['trajectory'].joint_trajectory.points[0]
        _end_traj_point = _cartesian_plan_handle['trajectory'].joint_trajectory.points[-1]
        _mse = rsmod.MSE_joint_points(_start_traj_point, _end_traj_point)
        print(f"{_client.move_group_name_} ->Attempt: {_attempt+1} -> MSE: {round(_mse,4)}")

        if _mse <= max_motion_threshold:
            return _cartesian_plan_handle
        print(f"Retrying...")
    print(f"Max attempts reached. Could not achieve MSE <= {max_motion_threshold}. Exiting...")
    return None

def get_mse_planend_current(_client, plan_handle):
    _current_joint_state = _client.get_current_joint_state()
    _target_joint_state = copy.deepcopy(_current_joint_state)
    _target_joint_state.position = plan_handle['trajectory'].joint_trajectory.points[-1].positions
    return rsmod.MSE_joint_states(_current_joint_state, _target_joint_state)


def main():
    rclpy.init()
    kg =None; kb = None
    
    kg =  MoveitInterface(node_name=f"client_kuka_blue",     
                                  move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )




    file_path = 'zigzag_path.csv'
    #while not input is to quit

    # Define the maximum motion threshold for the Cartesian planner
    CARTESIAN_MSE_THRESHOLD = 1.5
    

    
    _data_chisel = read_waypoints_csv(file_path)
    _chisel_start_point = _data_chisel[0]
    _chisel_start_pose = rosm.TxyzQxyzw_2_Pose(_chisel_start_point)
    # lift chisel in z +10 cm
    _chisel_home_point = copy.deepcopy(_chisel_start_point)
    _chisel_home_point[2] += 0.1  # lift chisel in z +10 cm
    _chisel_home_pose = rosm.TxyzQxyzw_2_Pose(_chisel_home_point)

    if kg: kg_plan_handle_home = plan_client_cartesian(kg, [_chisel_home_pose], CARTESIAN_MSE_THRESHOLD, 5)

    # # Execute both trajectories simultaneously
    # EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()
        
    # if EXECUTE_FLAG == "y":
    #     if kg: kg.execute_joint_traj(kg_plan_handle_home['trajectory'])

    #     _tick = time.time()
    #     execution_finished = False
    #     while not execution_finished:
    #         mse_kg = 0
    #         if kg: mse_kg = get_mse_planend_current(kg, kg_plan_handle_home)

    #         if (mse_kg < 0.0002): execution_finished = True
            
    #         _tock = time.time()
    #         if _tock - _tick > 60: print("Timeout: Execution not finished"); break
    #         time.sleep(0.01)

    
    # if kg: kg_plan_handle_start = plan_client_cartesian(kg, [_chisel_start_pose], CARTESIAN_MSE_THRESHOLD, 5)
    #     # Execute both trajectories simultaneously
    # EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()
        
    # if EXECUTE_FLAG == "y":
    #     if kg: kg.execute_joint_traj(kg_plan_handle_start['trajectory'])

    #     _tick = time.time()
    #     execution_finished = False
    #     while not execution_finished:
    #         mse_kg = 0
    #         if kg: mse_kg = get_mse_planend_current(kg, kg_plan_handle_start)

    #         if (mse_kg < 0.0002): execution_finished = True
            
    #         _tock = time.time()
    #         if _tock - _tick > 60: print("Timeout: Execution not finished"); break
    #         time.sleep(0.01)

    FRACTION_TO_RUN = 1.0
    SLOWNESS_FACTOR = 1.0

    # define int(FRACTION_TO_RUN*len(_data_chisel)
    index_ = int(FRACTION_TO_RUN*len(_data_chisel))

    _data_points_chisel = _data_chisel[:index_]


    print(f"Number of data points for chisel: {len(_data_points_chisel)}")

    _pose_waypoints_chisel = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_chisel)
    _pose_waypoints_chisel = _pose_waypoints_chisel.tolist()


    
    
    if kg: kg_plan_handle = plan_client_cartesian(kg, _pose_waypoints_chisel, CARTESIAN_MSE_THRESHOLD, 5)

    

    # Execute both trajectories simultaneously
    EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()
        
    if EXECUTE_FLAG == "y":
        if kg: kg.execute_joint_traj(kg_plan_handle['trajectory'])

        _tick = time.time()
        execution_finished = False
        while not execution_finished:
            mse_kg = 0
            if kg: mse_kg = get_mse_planend_current(kg, kg_plan_handle)

            if (mse_kg < 0.0002): execution_finished = True
            
            _tock = time.time()
            if _tock - _tick > 60: print("Timeout: Execution not finished"); break
            time.sleep(0.01)


    rclpy.shutdown()

if __name__ == '__main__':
    import sys

    main()
