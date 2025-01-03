#NOT WORKING
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

###############################
#---- Next Action Generator ---
###############################
_file_name = "ft_010.csv"
path = f'no-sync/edge_3/{_file_name}'
data_loader = cfp.DataParser.from_quat_file(file_path = path, target_fps= 60, filter=False, window_size=30, polyorder=3)
#while not input is to quit
FRACTION_TO_RUN = 1.0
SLOWNESS_FACTOR = 5.0


_data_chisel = data_loader.get_rigid_TxyzQwxyz()['chisel']
_data_gripper = data_loader.get_rigid_TxyzQwxyz()['gripper']

# define int(FRACTION_TO_RUN*len(_data_chisel)
index_ = int(FRACTION_TO_RUN*len(_data_chisel))

_data_chisel = _data_chisel[:index_]
_data_gripper = _data_gripper[:index_]
_data_times = data_loader.get_time()[:index_]

print(f"Number of data points for chisel: {len(_data_chisel)}")
print(f"Number of data points for gripper: {len(_data_gripper)}")
print(f"Number of data points for time: {len(_data_times)}")
#-----------------------------#






def get_robot_next_actions():
    step_size = 8
    # Zip the data_chisel and data_gripper so we can iterate over them simultaneously
    for i in range(0, len(_data_chisel), step_size):
        data_chisel_chunk = _data_chisel[i:i+step_size]
        data_gripper_chunk = _data_gripper[i:i+step_size]
        
        # Yield the chunks for both chisel and gripper
        yield {
            'data_chisel': data_chisel_chunk,
            'data_gripper': data_gripper_chunk
        }

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
    for _attempt in range(max_attemps):
        _cartesian_plan_handle = _client.get_cartesian_spline_plan(
            waypoints=waypoints, planning_frame='world',
            _planner_type="cartesian_interpolator", max_step=0.01,
            jump_threshold=0.0, avoid_collisions=False, attempts=1,
            max_velocity_scaling_factor = 0.05
        )
        
        
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

    

KG_HOME = [-0.614, 0.634, 2.302, -1.634, 1.526, -1.549, -1.897]
KB_HOME = [-0.590, -1.136, -2.251, 1.250, -1.929, 0.964, 0.494]

KG_CHISEL_START = [-0.908, 1.000, 2.218, -1.330, 1.377, -1.391, -2.146]
KB_GRIPPER_START = [-0.548, -0.289, -1.942, 1.609, -1.596, 1.258, -0.877]

###############################
#------------ Main -----------#
###############################
def main():
    rclpy.init()
    ###############################
    #------ Create Clients -------#
    ###############################

    kg =None; kb = None    
    kg = MoveitInterface(node_name=f"client_real_kuka_green",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_green",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )


    # kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
    #                               move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
    #                               remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
    #                               prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                              )

    # kg = MoveitInterface(node_name=f"client_sim_kuka_green",     
    #                                 move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
    #                                 remapping_name="",           # lbr # ""          #-> required for service and action remapping
    #                                 prefix="kuka_green",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                                 )


    # kb = MoveitInterface(node_name=f"client_sim_kuka_blue",     
    #                             move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
    #                             remapping_name="",           # lbr # ""          #-> required for service and action remapping
    #                             prefix="kuka_blue",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                             )

    #-----------------------------#
    
    action_generator = get_robot_next_actions()

    if kg: move_client_ptp(kg, KG_HOME)
    if kb: move_client_ptp(kb, KB_HOME)
    
    if kg: move_client_ptp(kg, KG_CHISEL_START)
    if kb: move_client_ptp(kb, KB_GRIPPER_START)

    for next_actions in action_generator:
        _data_points_chisel = next_actions['data_chisel']
        _data_points_chisel = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_chisel)
        _pose_waypoints_chisel = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_chisel)
        _pose_waypoints_chisel = _pose_waypoints_chisel.tolist()

        _data_points_gripper = next_actions['data_gripper']
        _data_points_gripper = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_gripper)
        _pose_waypoints_gripper = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_gripper)
        _pose_waypoints_gripper = _pose_waypoints_gripper.tolist()

        CARTESIAN_MSE_THRESHOLD = 0.1
        
        if kg: kg_plan_handle = plan_client_cartesian(kg, _pose_waypoints_chisel, CARTESIAN_MSE_THRESHOLD, 5)
        if kb: kb_plan_handle = plan_client_cartesian(kb, _pose_waypoints_gripper, CARTESIAN_MSE_THRESHOLD, 5)


        EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()
        
        if EXECUTE_FLAG == input("Execute trajectory? (y/n): ").strip().lower():
            if kg: kg.execute_joint_traj(kg_plan_handle['trajectory'])
            if kb: kb.execute_joint_traj(kb_plan_handle['trajectory'])

            _tick = time.time()
            execution_finished = False
            while not execution_finished:
                mse_kg = 0
                if kg: mse_kg = get_mse_planend_current(kg, kg_plan_handle)

                mse_kb = 0
                if kb: mse_kb = get_mse_planend_current(kb, kb_plan_handle)

                if (mse_kg < 0.0002) and (mse_kb < 0.0002): execution_finished = True
                
                _tock = time.time()
                if _tock - _tick > 10: print("Timeout: Execution not finished"); break

                time.sleep(0.01)

    if kg: move_client_ptp(kg, KG_HOME)
    if kb: move_client_ptp(kb, KB_HOME)
    rclpy.shutdown()

if __name__ == '__main__':
    import sys

    # _file_name = sys.argv[1] if len(sys.argv) > 1 else "ft_010.csv"

    main()
