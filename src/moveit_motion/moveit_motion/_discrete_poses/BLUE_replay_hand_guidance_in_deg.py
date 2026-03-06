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
import time
import csv
from math import pi
import moveit_motion.ros_submodules.RS_submodules as rsmod
import re


KB_HOME = [2.3038370893638067, 1.286546810606163, 0.9388634138002693, 1.4017270163924056, 0.3789394657776249, -1.1635984302260418, -1.5263421627553195]


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
            waypoints=waypoints, planning_frame='world',
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
    

import re

def parse_robot_data(file_path):
    """
    Parses the robot data file and returns selected fields as numpy arrays.
    
    Parameters:
        file_path (str): Path to the data file.
    
    Returns:
        dict[str, np.ndarray]: time, J1..J7, Fx/Fy/Fz, Tx/Ty/Tz arrays.
    """
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('%'):
                header = line.strip().lstrip('%').strip().split()
                break
        else:
            raise ValueError("No header line starting with '%' found in the file.")

    data = np.loadtxt(file_path, comments='%', ndmin=2)
    if data.size == 0:
        raise ValueError(f"No numeric rows found in {file_path}")

    # Map each header token to its column index once.
    col_idx = {name: i for i, name in enumerate(header)}

    joint_pattern = re.compile(r'axisQMsr_LBR_iiwa_7_R800_1\[(\d+)\]')
    joint_columns = [col for col in header if joint_pattern.match(col)]
    if len(joint_columns) != 7:
        raise ValueError(f"Expected 7 joint columns, found {len(joint_columns)}.")

    joint_columns_sorted = sorted(
        joint_columns,
        key=lambda x: int(joint_pattern.match(x).group(1))
    )

    if 'ZeitInSec' not in col_idx or 'ZeitInNanoSec' not in col_idx:
        raise ValueError("Required time columns 'ZeitInSec' and/or 'ZeitInNanoSec' are missing.")

    time_sec = data[:, col_idx['ZeitInSec']]
    time_ns = data[:, col_idx['ZeitInNanoSec']]
    out = {'time': time_sec + time_ns * 1e-9}

    for j, source_col in enumerate(joint_columns_sorted, start=1):
        out[f'J{j}'] = data[:, col_idx[source_col]]

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
    
    missing_force_columns = [col for col in force_columns.keys() if col not in col_idx]
    missing_torque_columns = [col for col in torque_columns.keys() if col not in col_idx]
    missing_columns = missing_force_columns + missing_torque_columns
    if missing_columns:
        raise ValueError(f"The following required columns are missing in the data: {missing_columns}")

    for source_col, target_col in force_columns.items():
        out[target_col] = data[:, col_idx[source_col]]
    for source_col, target_col in torque_columns.items():
        out[target_col] = data[:, col_idx[source_col]]

    return out

def read_joint_states_from_csv(file_path):
    joint_states = []
    force_torques = []

    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            joint_states.append([(float(row['J1'])), (float(row['J2'])), (float(row['J3'])),
                                 (float(row['J4'])), (float(row['J5'])), (float(row['J6'])), (float(row['J7']))])
            
            force_torques.append([(float(row['fx'])), (float(row['fy'])), (float(row['fz'])),
                                 (float(row['tx'])), (float(row['ty'])), (float(row['tz']))])
    return joint_states, force_torques


def get_robot_next_actions(robot_data, step_size=8):
    # Zip the data_chisel and data_gripper so we can iterate over them simultaneously
    for i in range(0, len(robot_data), step_size):
        data_next = robot_data[i:i+step_size]
        
        # Yield the chunks for both chisel and gripper
        yield data_next

def main_simple(robot_data):
    rclpy.init()

    kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
                                  move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    
    # action_generator = get_robot_next_actions()
    cjs = kb.get_current_joint_state()
    if kb: move_client_ptp(kb, KB_HOME)



    joint_columns = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7']
    joint_values_deg = np.column_stack([robot_data[col] for col in joint_columns])

    joint_values_rad = np.deg2rad(joint_values_deg)
    # joint_values_rad = joint_values_deg

    fjs = joint_values_rad[0]

    move_client_ptp(kb, fjs)
    
    input("Press Enter to continue...")

    # ##  ---------- method 1 ------------


    joint_times = robot_data['time'].copy()
    joint_times = joint_times - joint_times[0]
    # joint_times = None

    

    joint_trajectory_msg = rosm.joint_points_2_trajectory(
        points=joint_values_rad[0:],
        times = joint_times,
        header_frame_id = 'world',
        joint_names = cjs.name,
        sampling_rate = 10
    )

    kb.execute_joint_traj(joint_trajectory_msg)

    rclpy.shutdown()



# Example Usage
if __name__ == "__main__":
    import sys
    from pathlib import Path
    from datetime import datetime

    def latest_log_file(directory: Path) -> Path:
        logs = list(directory.glob("*.log"))
        if not logs:
            raise FileNotFoundError(f"No .log files found in {directory}")
        # Prefer filename order (your logs are YYYY-MM-DD_HH-MM-SS.log).
        # KUKA controller clock can be wrong (mtime resets), so don't rely on mtime here.
        return max(logs, key=lambda p: p.name)

    data_directory = Path("/mnt/rec_kuka_blue")

    # Usage:
    #   - no args: use latest .log in /mnt/rec_kuka_blue
    #   - "latest": same as above
    #   - "<full/path/to/file.log>": use that file
    arg = sys.argv[1].strip() if len(sys.argv) > 1 else "latest"
    if arg == "" or arg.lower() == "latest":
        data_file = latest_log_file(data_directory)
    else:
        data_file = Path(arg).expanduser()
        if not data_file.is_absolute():
            raise ValueError("Pass a full path, or 'latest'.")
        if not data_file.exists():
            raise FileNotFoundError(f"Log file not found: {data_file}")

    mtime = datetime.fromtimestamp(data_file.stat().st_mtime).strftime("%Y-%m-%d %I:%M:%S %p")
    print(f"""
        --------------------------------------
        -- Replaying log:
        -- {data_file}
        --------------------------------------
        """
        )
    # print(f"Replaying log: {data_file} (mtime: {mtime})")

    
    START_INDEX = 0  # Skip the first few rows of the data file
    
    try:
        robot_data = parse_robot_data(str(data_file))
        # Keep parity with previous "skip initial rows" behavior.
        robot_data = {k: v[START_INDEX:] for k, v in robot_data.items()}
        # print("Parsed Data:")
        # print({k: v[:5] for k, v in robot_data.items()})  # Preview first rows
    except Exception as e:
        print(f"An error occurred: {e}")

    main_simple(robot_data)
    # main_cartesian_planning()
