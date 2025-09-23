import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface
import copy
import moveit_motion.ros_submodules.ros_math as rosm
import numpy as np
import moveit_motion.diffusion_policy_cam.submodules.robomath_addon as rma
import moveit_motion.diffusion_policy_cam.submodules.robomath as rm
import moveit_motion.diffusion_policy_cam.submodules.data_filter as dft
import time
import csv
from math import pi
import pandas as pd
import moveit_motion.ros_submodules.RS_submodules as rsmod
import pandas as pd
import re


KB_HOME = [1.464, 1.273, -1.529, -1.352, 1.068, 0.673, 0.28]



def move_client_ptp(_client, goal_list: list, tolerance=0.00005, time_out=60):
    _cjs = _client.get_current_joint_state()
    _goal_state = rosm.joint_list_2_state(goal_list, _cjs.name)

    if rsmod.MSE_joint_states(_cjs, _goal_state) < tolerance:
        print("Already at goal")
        return
    
    _goal_plan_handle = _client.get_joint_ptp_plan(_cjs, _goal_state, max_velocity_scaling_factor=0.1)
    
    # condition = 'y'
    condition = input(f"Execute {_client.move_group_name_} plan? (y/n): ").strip().lower()
    if condition == 'y':
        _client.execute_joint_traj(_goal_plan_handle['trajectory'])

        _tick = time.time()
        execution_finished = False
        print("Waiting for execution to finish...")
        while not execution_finished:
            mse_client = get_mse_planend_current(_client, _goal_plan_handle['trajectory'])

            if (mse_client < 0.0002):
                execution_finished = True
            
            _tock = time.time()

            if _tock - _tick > time_out: print(f"Timeout {time_out} seconds: Execution not finished"); break
            time.sleep(0.01)




def plan_client_cartesian(_client, waypoints: list, 
                          max_motion_threshold= float, max_attemps: int = 1,
                          **kwargs):
    planner_type = "cartesian_interpolator"
    # planner_type = "cartesian_sequence_action"
    
    '''
    motion-plan-specific parameters:
        num_planning_attempts = 10, 
        allowed_planning_time=10.0, 
        max_velocity_scaling_factor=0.1,
        max_acceleration_scaling_factor=0.1,
        pipeline_id,
        planner_id,
        goal_constraints, path_constraints
        start_state

    pilz-sequence-action-specific parameters:
        blend_radius=0.000005
    
    cartesian-interpolator-specific parameters:
        max_step = 0.01 # Decreased from default to increase waypoints
        jump_threshold = 0.0 # Disable to prevent sudden jumps
        avoid_collisions = False,
        'prismatic_jump_threshold': 0.1, 
        'revolute_jump_threshold': 0.1, -> By setting the threshold to 0.1 radians, you are allowing a maximum joint angle change of about 5.73 degrees between consecutive waypoints for any revolute joint.
    '''
    
    for _attempt in range(max_attemps):
        _cartesian_plan_handle = _client.get_cartesian_spline_plan(
            waypoints=waypoints, 
            planning_frame=kwargs.get('planning_frame', 'world'),
            attempts=10, 
            _planner_type=planner_type,
            #motion-plan-specific parameters
            allowed_planning_time=kwargs.get('allowed_planning_time', 10.0), 
            max_velocity_scaling_factor=kwargs.get('max_velocity_scaling_factor', 0.1),
            max_acceleration_scaling_factor=kwargs.get('max_acceleration_scaling_factor', 0.1),
            num_planning_attempts=kwargs.get('num_planning_attempts', 100),
            
            #pilz-sequence-action-specific parameters
            blend_radius=kwargs.get('blend_radius', 0.000005),
            
            # cartesian-interpolator-specific parameters
            max_step = kwargs.get("max_step", 0.01),
            jump_threshold = kwargs.get("jump_threshold", 0.0),
            avoid_collisions = kwargs.get("avoid_collisions", False),
            revolute_jump_threshold = kwargs.get("revolute_jump_threshold", 0.0),
            prismatic_jump_threshold = kwargs.get("prismatic_jump_threshold", 0.0),
        )
        
        slowness_factor = kwargs.get('slowness_factor', 1)
        if slowness_factor:
            _cartesian_plan_handle['trajectory'] = rosm.slow_down_trajectory(_cartesian_plan_handle['trajectory'], slowness_factor)
            
        if planner_type == "cartesian_sequence_action":
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


def get_mse_planend_current(_client, _trajectory):
    _current_joint_state = _client.get_current_joint_state()
    _target_joint_state = copy.deepcopy(_current_joint_state)
    _target_joint_state.position = _trajectory.joint_trajectory.points[-1].positions
    return rsmod.MSE_joint_states(_current_joint_state, _target_joint_state)
    


def parse_robot_data(file_path):
    """
    Parses the robot data file, renames joint columns to J1-J7,
    and extracts force and torque values.

    Parameters:
        file_path (str): Path to the data file.

    Returns:
        pd.DataFrame: A DataFrame containing renamed joint values and force/torque data.
    """
    # Step 1: Read the header line starting with '%'
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('%'):
                header = line.strip().lstrip('%').strip().split()
                break
        else:
            raise ValueError("No header line starting with '%' found in the file.")
    
    # Step 2: Load the data into a DataFrame
    df = pd.read_csv(
        file_path,
        delim_whitespace=True,    # Assuming the data is space-separated
        comment='%',              # Skip any lines starting with '%'
        names=header,             # Use the extracted header
        skiprows=1                # Skip the header line
    )
    
    # Step 3: Identify and rename joint columns
    joint_pattern = re.compile(r'axisQMsr_LBR_iiwa_7_R800_1\[(\d+)\]')
    joint_columns = [col for col in header if joint_pattern.match(col)]
    
    if len(joint_columns) != 7:
        raise ValueError(f"Expected 7 joint columns, found {len(joint_columns)}.")
    
    # Sort joint columns based on their index and rename them to J1-J7
    joint_columns_sorted = sorted(
        joint_columns,
        key=lambda x: int(joint_pattern.match(x).group(1))
    )
    joint_rename_map = {col: f'J{idx+1}' for idx, col in enumerate(joint_columns_sorted)}
    df.rename(columns=joint_rename_map, inplace=True)
    
    # Step 4: Extract force and torque columns
    force_columns = ['cartForce1_X', 'cartForce1_Y', 'cartForce1_Z']
    torque_columns = ['cartTorque1_TauX', 'cartTorque1_TauY', 'cartTorque1_TauZ']
    
    # Verify that the required columns exist
    missing_columns = [col for col in force_columns + torque_columns if col not in df.columns]
    if missing_columns:
        raise ValueError(f"The following required columns are missing in the data: {missing_columns}")
    
    # Step 5: Select the renamed joint columns along with force and torque data
    selected_columns = list(joint_rename_map.values()) #+ force_columns + torque_columns
    df_selected = df[selected_columns]
    
    return df_selected




def parse_robot_data(file_path):
    """
    Parses the robot data file, renames joint columns to J1-J7,
    extracts and renames time, force, and torque values.
    
    Parameters:
        file_path (str): Path to the data file.
    
    Returns:
        pd.DataFrame: A DataFrame containing time, renamed joint values,
                      and renamed force/torque data.
    """
    # Step 1: Read the header line starting with '%'
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('%'):
                header = line.strip().lstrip('%').strip().split()
                break
        else:
            raise ValueError("No header line starting with '%' found in the file.")
    
    # Step 2: Load the data into a DataFrame
    df = pd.read_csv(
        file_path,
        delim_whitespace=True,    # Assuming the data is space-separated
        comment='%',              # Skip any lines starting with '%'
        names=header,             # Use the extracted header
        skiprows=1                # Skip the header line
    )
    
    # Step 3: Identify and rename joint columns
    joint_pattern = re.compile(r'axisQMsr_LBR_iiwa_7_R800_1\[(\d+)\]')
    joint_columns = [col for col in header if joint_pattern.match(col)]
    
    if len(joint_columns) != 7:
        raise ValueError(f"Expected 7 joint columns, found {len(joint_columns)}.")
    
    # Sort joint columns based on their index and rename them to J1-J7
    joint_columns_sorted = sorted(
        joint_columns,
        key=lambda x: int(joint_pattern.match(x).group(1))
    )
    joint_rename_map = {col: f'J{idx+1}' for idx, col in enumerate(joint_columns_sorted)}
    df.rename(columns=joint_rename_map, inplace=True)
    
    # Step 4: Extract and combine time columns
    if 'ZeitInSec' not in df.columns or 'ZeitInNanoSec' not in df.columns:
        raise ValueError("Required time columns 'ZeitInSec' and/or 'ZeitInNanoSec' are missing.")
    
    # Combine 'ZeitInSec' and 'ZeitInNanoSec' into a single 'time' column in seconds
    df['time'] = df['ZeitInSec'] + df['ZeitInNanoSec'] * 1e-9
    
    # Drop the original time columns as they are now combined
    df.drop(columns=['ZeitInSec', 'ZeitInNanoSec'], inplace=True)
    
    # Step 5: Extract and rename force and torque columns
    force_columns = {
        'cartForce1_X': 'Fx',
        'cartForce1_Y': 'Fy',
        'cartForce1_Z': 'Fz'
    }
    torque_columns = {
        'cartTorque1_TauX': 'Tx',
        'cartTorque1_TauY': 'Ty',
        'cartTorque1_TauZ': 'Tz'
    }
    
    # Verify that the required force and torque columns exist
    missing_force_columns = [col for col in force_columns.keys() if col not in df.columns]
    missing_torque_columns = [col for col in torque_columns.keys() if col not in df.columns]
    missing_columns = missing_force_columns + missing_torque_columns
    if missing_columns:
        raise ValueError(f"The following required columns are missing in the data: {missing_columns}")
    
    # Rename force and torque columns
    df.rename(columns=force_columns, inplace=True)
    df.rename(columns=torque_columns, inplace=True)
    
    # Step 6: Select the relevant columns
    selected_columns = ['time'] + list(joint_rename_map.values()) + list(force_columns.values()) + list(torque_columns.values())
    df_selected = df[selected_columns]
    
    return df_selected


def get_robot_next_actions(robot_data, step_size=8):
    # Zip the data_chisel and data_gripper so we can iterate over them simultaneously
    for i in range(0, len(robot_data), step_size):
        data_next = robot_data[i:i+step_size]
        
        # Yield the chunks for both chisel and gripper
        yield data_next


def read_joint_states_from_csv(file_path):
    joint_states = []
    force_torques = []

    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            joint_states.append([(float(row['J1'])), (float(row['J2'])), (float(row['J3'])),
                                 (float(row['J4'])), (float(row['J5'])), (float(row['J6'])), (float(row['J7']))])
            
            force_torques.append([(float(row['X'])), (float(row['fy'])), (float(row['fz'])),
                                 (float(row['tx'])), (float(row['ty'])), (float(row['tz']))])
    return joint_states, force_torques


def read_TxyzQwxyz_FxyzMxyz(file_path):
    joint_states = []
    force_torques = []

    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            joint_states.append([(float(row['X'])), (float(row['Y'])), (float(row['Z'])),
                                 (float(row['Qw'])), (float(row['Qx'])), (float(row['Qy'])), (float(row['Qz']))])
            
            force_torques.append([(float(row['Fx'])), (float(row['Fy'])), (float(row['Fz'])),
                                 (float(row['Mx'])), (float(row['My'])), (float(row['Mz']))])
    return joint_states, force_torques


def main_simple():
    rclpy.init()
    import os
    kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
                                move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                )
    
    
    cjs = kb.get_current_joint_state()


    ###### Read waypoints from csv file ######
    #-----------------------------------------
    file_path = "no-sync/replay_traj_data/traj_178/cartesian_waypoints.csv"

    # -----------------------------------------



    waypoints, force_torques = read_TxyzQwxyz_FxyzMxyz(file_path)

    waypoints_0 = rosm.robodk_2_ros(waypoints[0])
    pose_0 = rosm.TxyzQxyzw_2_Pose(waypoints_0)
    joint_values_0 = kb.get_best_ik(current_joint_state=cjs, target_pose=pose_0, attempts=10)

    move_client_ptp(kb, joint_values_0.position, tolerance=0.00005, time_out=60)

    action_generator = get_robot_next_actions(waypoints[1:], step_size=20)

    for action in action_generator:
        _data_points_gripper = action
        # _data_points_gripper = dft.smooth_waypoints_sg(_data_points_gripper, window_length=10, polyorder=3)
        _data_points_gripper = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_gripper)
        _pose_waypoints_gripper = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_gripper)
        _pose_waypoints_gripper = _pose_waypoints_gripper.tolist()

        CARTESIAN_MSE_THRESHOLD = 0.8
        
            
        if kb: kb_plan_handle = plan_client_cartesian(kb, _pose_waypoints_gripper, CARTESIAN_MSE_THRESHOLD, max_attemps=1, 
                                                          slowness_factor=SLOWNESS_FACTOR, planning_frame=kb.base_)

        EXECUTE_FLAG = 'y'
        
        if EXECUTE_FLAG == 'y': # input("Execute trajectory? (y/n): ").strip().lower():
            if kb: kb.execute_joint_traj(kb_plan_handle['trajectory'])

            _tick = time.time()
            execution_finished = False
            while not execution_finished:

                mse_kb = 0
                if kb: mse_kb = get_mse_planend_current(kb, kb_plan_handle['trajectory'])

                if (mse_kb < 0.0002): execution_finished = True
                
                _tock = time.time()
                if _tock - _tick > EXECUTION_TIMEOUT: print(f"Timeout {EXECUTION_TIMEOUT} seconds: Execution not finished"); break

                time.sleep(0.01)

    print("Going to home position")

        
    rclpy.shutdown()



# Example Usage
if __name__ == "__main__":
    import sys
    STEP_SIZE = 60
    FRACTION_TO_RUN = 1.0
    SLOWNESS_FACTOR = 3
    EXECUTION_TIMEOUT = 10
    main_simple()
