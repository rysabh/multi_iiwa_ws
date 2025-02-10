import rclpy
import numpy as np
import torch
import torch.nn as nn
import collections
from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface
import copy
from rclpy.executors import MultiThreadedExecutor
import moveit_motion.ros_submodules.ros_math as rosm
import moveit_motion.diffusion_policy_cam.submodules.robomath_addon as rma
import moveit_motion.diffusion_policy_cam.diffusion_pipline.data_processing as dproc
import moveit_motion.diffusion_policy_cam.diffusion_pipline.model as md
import moveit_motion.diffusion_policy_cam.MSEC.vision_encoder as ve
from ati_sensor_service.ati_client import AtiClient
import threading
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler
import time
import pyrealsense2 as rs
import pandas as pd
import moveit_motion.ros_submodules.RS_submodules as rsmod
import re
import cv2
from matplotlib import pyplot as plt


def predicted_action(obs_deque, statistics, obs_horizon,
                        pred_horizon, action_horizon, action_dim, 
                        noise_scheduler, num_diffusion_iters,
                        ema_noise_pred_net, device):   
    
    
    stats = statistics

    # save visualization and rewards
    B = 1
    # stack the last obs_horizon (2) number of observations
    obs_seq = np.stack(obs_deque)
    # normalize observation
    nobs = dproc.normalize_data(obs_seq, stats=stats['obs'])
    # device transfer
    nobs = torch.from_numpy(nobs).to(device, dtype=torch.float32)
    # infer action
    with torch.no_grad():
        # reshape observation to (B,obs_horizon*obs_dim)
        obs_cond = nobs.unsqueeze(0).flatten(start_dim=1)
        
        # initialize action from Guassian noise
        noisy_action = torch.randn(
            (B, pred_horizon, action_dim), device=device)
        naction = noisy_action

        # init scheduler
        noise_scheduler.set_timesteps(num_diffusion_iters)

        for k in noise_scheduler.timesteps:
            # predict noise
            noise_pred = ema_noise_pred_net(
                sample=naction,
                timestep=k,
                global_cond=obs_cond
            )

            # inverse diffusion step (remove noise)
            naction = noise_scheduler.step(
                model_output=noise_pred,
                timestep=k,
                sample=naction
            ).prev_sample

    # unnormalize action
    naction = naction.detach().to('cpu').numpy()
    naction = naction[0]
    action_pred = dproc.unnormalize_data(naction, stats=stats['action'])

    # only take action_horizon number of actions
    start = obs_horizon - 1
    end = start + action_horizon
    action = action_pred[start:end,:]
    # grouped_actions = {name: [] for name in action_item}

    # # Divide the 18 elements in each row into three groups
    # for i in range(len(action)):
    #     for j in range(len(action_item)):
    #         grouped_actions[action_item[j]].append(action[i, 6*j:6*(j+1)])
    #         grouped_actions[action_item[j]].append(action[i, -1])

    return action


def _model_initalization(action_dim, obs_dim, obs_horizon, pred_horizon, num_epochs, len_dataloader, num_diffusion_iters) -> nn.Module:
    # create network object
    noise_pred_net = md.ConditionalUnet1D(
        input_dim=action_dim,
        global_cond_dim=obs_dim*obs_horizon
    )

    # example inputs
    noised_action = torch.randn((1, pred_horizon, action_dim))
    obs = torch.zeros((1, obs_horizon, obs_dim))
    diffusion_iter = torch.zeros((1,))

    # the noise prediction network
    # takes noisy action, diffusion iteration and observation as input
    # predicts the noise added to action
    noise = noise_pred_net(
        sample=noised_action,
        timestep=diffusion_iter,
        global_cond=obs.flatten(start_dim=1))

    # illustration of removing noise
    # the actual noise removal is performed by NoiseScheduler
    # and is dependent on the diffusion noise schedule
    denoised_action = noised_action - noise

    # for this demo, we use DDPMScheduler with 100 diffusion iterations
    # num_diffusion_iters = 100
    noise_scheduler = DDPMScheduler(
        num_train_timesteps=num_diffusion_iters,
        # the choise of beta schedule has big impact on performance
        # we found squared cosine works the best
        beta_schedule='squaredcos_cap_v2',
        # clip output to [-1,1] to improve stability
        clip_sample=True,
        # our network predicts noise (instead of denoised action)
        prediction_type='epsilon'
    )

    # device transfer
    device = torch.device('cuda')
    _ = noise_pred_net.to(device)

    # Exponential Moving Average
    # accelerates training and improves stability
    # holds a copy of the model weights
    ema = EMAModel(
        parameters=noise_pred_net.parameters(),
        power=0.75)

    # Standard ADAM optimizer
    # Note that EMA parametesr are not optimized
    optimizer = torch.optim.AdamW(
        params=noise_pred_net.parameters(),
        lr=1e-4, weight_decay=1e-6)

    # Cosine LR schedule with linear warmup
    lr_scheduler = get_scheduler(
        name='cosine',
        optimizer=optimizer,
        num_warmup_steps=200,
        num_training_steps=len_dataloader * num_epochs
    )

    ema_noise_pred_net = noise_pred_net

    return ema_noise_pred_net, noise_scheduler, device, ema, optimizer, lr_scheduler


def move_client_ptp(_client, goal_list: list, current_joint_state=None, tolerance=0.00005, time_out=60):
    _cjs = current_joint_state
    
    if _cjs is None:
        _cjs = _client.get_current_joint_state()
    
    _goal_state = rosm.joint_list_2_state(goal_list, _cjs.name)
    
    if rsmod.MSE_joint_states(_cjs, _goal_state) < tolerance:
        print("move_client_ptp: Already at goal")
        return
    
    _goal_plan_handle = _client.get_joint_ptp_plan(_cjs, _goal_state, max_velocity_scaling_factor=0.1)
    
    condition = 'y'
    # condition = input(f"Execute {_client.move_group_name_} plan? (y/n): ").strip().lower()
    if condition == 'y':
        _client.execute_joint_traj(_goal_plan_handle['trajectory'])

        _tick = time.time()
        execution_finished = False
        print("move_client_ptp: Waiting for execution to finish...")
        while not execution_finished:
            mse_client = get_mse_planend_current(_client, _goal_plan_handle['trajectory'])

            if (mse_client < 0.0002):
                execution_finished = True
            
            _tock = time.time()

            if _tock - _tick > time_out: print(f"move_client_ptp: Timeout {time_out} seconds: Execution not finished"); break
            time.sleep(0.01)
        
        if execution_finished:
            print("move_client_ptp: move_client_ptp: Execution finished")


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

def move_robot(action, kb):

    # Get the current joint state
    _cjs = kb.get_current_joint_state()

    # Perform the movement of the robot in this thread
    move_client_ptp(kb, goal_list=action[0], current_joint_state=_cjs, tolerance=MOTION_TOLERANCE)

    joint_trajectory_msg = rosm.joint_points_2_trajectory(
        points=action[1:],
        times=None,
        header_frame_id='world',
        joint_names=_cjs.name,
        sampling_rate=0.1
    )

    print("Executing trajectory...")
    kb.execute_joint_traj(joint_trajectory_msg)
    print("Executed trajectory")

def get_observations(obs_deque, kb, ati_client, pipeline, stop_event, last_valid_joint_state=None):
    # Continuous observation collection in this thread
    response_kuka = []

    while not stop_event.is_set():  # Check if the stop event is set
        start_time = time.time()  # Record the start time of the loop iteration

        # Get the current joint state
        action_observation = None
        last_valid_joint_state = None
        while action_observation is None:
            action_observation = kb.get_current_joint_state()
            if action_observation is None:
                print("Warning: Failed to fetch current joint state, retrying...")
                if last_valid_joint_state is not None:
                    print("Using last valid joint state as fallback.")
                    action_observation = last_valid_joint_state 
                time.sleep(0.015)  # Wait for a short time before retrying

        # Save the current valid joint state for future fallback
        last_valid_joint_state = action_observation

        # Initialize the Image observation
        image_observation = None
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image_resized = np.asanyarray(color_frame.get_data(), dtype=np.float32)
        image_observation = cv2.resize(image_resized, (240, 240))

        # Get the force torque data
        force_observations = None
        last_valid_force_state = None
        while force_observations is None:
            force_observations = ati_client.get_force_torque()
            force_thread = threading.Thread(target=force_observation_thread, args=(ati_client, force_observations))
            force_thread.start()
            force_thread.join()
            if not force_observations.done():
                print("Warning: Force/Torque observation not ready, retrying...")
                if last_valid_force_state is not None:
                    print("Using last valid joint state as fallback.")
                    force_observations = last_valid_force_state
                time.sleep(0.1)

        last_valid_force_state = force_observations
        # force_observations = ati_client.get_force_torque()
        # rclpy.spin_until_future_complete(ati_client, force_observations)

        if action_observation is not None and force_observations.done() and image_observation is not None:
            response_kuka = action_observation.position
            response_ati = force_observations.result().msg
            force_values = [response_ati.fx, response_ati.fy, response_ati.fz, response_ati.tx, response_ati.ty, response_ati.tz]
            response_kuka.extend(force_values)

        # Calculate the time taken for the current iteration
        elapsed_time = time.time() - start_time
        print(f"Elapsed time: {elapsed_time}")
        sleep_time = 0.015 - elapsed_time  # Ensure the loop runs at approximately 0.03 sec intervals

        if sleep_time > 0:
            time.sleep(sleep_time)

        # Store the observation
        observation = {'image': image_observation, 'agent_pos': response_kuka}
        obs_deque.append(observation)

    print("Observation thread is stopping.")


def move_robot_get_obs_threads(action, obs_deque, kb, kb2, pipeline, ati_client):
    # Create a stop event to signal the observation thread when to stop
    stop_event = threading.Event()

    # Start the thread for getting observations
    obs_thread = threading.Thread(target=get_observations, args=(obs_deque, kb2, ati_client, pipeline, stop_event))
    obs_thread.start()

    # Start the thread for moving the robot
    move_thread = threading.Thread(target=move_robot, args=(action, kb))
    move_thread.start()
    
    # Wait for the move thread to finish
    move_thread.join()

    # After move_thread is done, stop the observation thread
    stop_event.set()  # This will signal the observation thread to stop
    
    # Wait for the observation thread to finish
    obs_thread.join()

    return obs_deque


def force_observation_thread(ati_client, force_observations):
    """Function to handle force torque retrieval in a separate thread."""
    # Use the MultiThreadedExecutor to spin the ati_client node and handle future completion
    executor = MultiThreadedExecutor()
    executor.add_node(ati_client)
    rclpy.spin_until_future_complete(ati_client, force_observations)
    executor.shutdown()

def move_robot_get_obs(action, obs_deque, kb, kb2, pipeline, ati_client):
    # Start the robot movement in a separate thread
    move_thread = threading.Thread(target=move_robot, args=(action, kb))
    move_thread.start()
    
    # Continuous observation collection while the robot is moving
    response_kuka = []

    # Track the last valid joint state and force state
    last_valid_joint_state = None

    # Loop while the move thread is running and collect observations every 0.015 seconds
    while move_thread.is_alive():
        loop_start_time = time.time()  # Record the start time of this loop iteration

        # Get the current joint state
        action_observation = None
        while action_observation is None:
            action_observation = kb2.get_current_joint_state()
            if action_observation is None:
                print("Warning: Failed to fetch current joint state, retrying...")
                if last_valid_joint_state is not None:
                    print("Using last valid joint state as fallback.")
                    action_observation = last_valid_joint_state
                time.sleep(0.015)  # Wait for a short time before retrying

        # Save the current valid joint state for future fallback
        last_valid_joint_state = action_observation

        # Initialize the Image observation
        image_observation = None
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image_resized = np.asanyarray(color_frame.get_data(), dtype=np.float32)
        image_observation = cv2.resize(image_resized, (240, 240))

        # Get the force torque data
        force_observations = ati_client.get_force_torque()
        # Use rclpy.spin_until_future_complete() to wait for force torque data
        # rclpy.spin_until_future_complete(ati_client, force_observations)
        # force_thread = threading.Thread(target=force_observation_thread, args=(ati_client, force_observations))
        # force_thread.start()
        # force_thread.join()
        print(force_observations.result().msg)

        # Check if force observations are complete
        if action_observation is not None and force_observations.done() and image_observation is not None:
            response_kuka = action_observation.position
            response_ati = force_observations.result().msg
            force_values = [response_ati.fx, response_ati.fy, response_ati.fz, response_ati.tx, response_ati.ty, response_ati.tz]
            response_kuka.extend(force_values)

        # Calculate the time taken for the current iteration
        elapsed_time = time.time() - loop_start_time
        print(f"Elapsed time: {elapsed_time}")
        
        # Sleep for the remainder of 0.015 seconds to maintain the desired frequency
        sleep_time = 0.015 - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Store the observation
        observation = {'image': image_observation, 'agent_pos': response_kuka}
        obs_deque.append(observation)

    # Ensure the last observation is appended after the robot finishes moving
    print("Robot movement finished, appending final observation.")
    print("Observation collection complete.")
    return obs_deque

def moving_robot_joint_ptp_obs(action, obs_deque, kb, ati_client, box_observation):

    for joint_values in action:

        move_client_ptp(kb, joint_values)
        response_kuka = []
        action_observation = kb.get_current_joint_state()

        # Get the force torque data
        force_observations = ati_client.get_force_torque()
        rclpy.spin_until_future_complete(ati_client, force_observations)

        if action_observation is not None and force_observations.done():
            response_kuka = action_observation.position
            response_ati = force_observations.result().msg
            force_values = [response_ati.fx, response_ati.fy, response_ati.fz, response_ati.tx, response_ati.ty, response_ati.tz]
            response_kuka.extend(box_observation)
            response_kuka.extend(force_values)

        # Store the observation
        # observation = {'action': np.array(action_observation.position, dtype=np.float32), 'obs': np.array(response_kuka, dtype=np.float32)}
        observation = [np.array(response_kuka, dtype=np.float32)]
        obs_deque.append(observation)

    return obs_deque

def moving_robot_cartesian_ptp_obs(action, obs_deque, kb, ati_client, box_observation):

    action_TxyzQwxyz = np.apply_along_axis(rma.TxyzRxyz_2_TxyzQwxyz, 1, action)
    action_robot = np.apply_along_axis(rosm.robodk_2_ros, 1, action_TxyzQwxyz)

    for cartesian_values in action_robot:

        ############################################################
        ####### change cartesian to joint value

        cjs = kb.get_current_joint_state()
        pose = rosm.TxyzQxyzw_2_Pose(cartesian_values)
        joint_values = kb.get_best_ik(current_joint_state=cjs, target_pose=pose, attempts=10)

        move_client_ptp(kb, joint_values.position)
        response_kuka = []
        action_observation = kb.get_current_joint_state()

        ############################################################33
        ### CHANGE THE VALUE To FK
        joint_msg_values = rosm.joint_list_2_state(joint_positions=action_observation.position, joint_names=action_observation.name)
        _fk = kb.get_fk(joint_state=joint_msg_values, header_frame_id=kb.base_)
        _Txyz_Qwxyz = [_fk.position.x, _fk.position.y, _fk.position.z, _fk.orientation.w, _fk.orientation.x, _fk.orientation.y, _fk.orientation.z]
        _Txyz_Rxyz = rma.TxyzQwxyz_2_TxyzRxyz(_Txyz_Qwxyz)


        # Get the force torque data
        force_observations = ati_client.get_force_torque()
        rclpy.spin_until_future_complete(ati_client, force_observations)

        if _Txyz_Rxyz is not None and force_observations.done():
            response_kuka = _Txyz_Rxyz
            response_ati = force_observations.result().msg
            force_values = [response_ati.fx, response_ati.fy, response_ati.fz, response_ati.tx, response_ati.ty, response_ati.tz]
            response_kuka.extend(box_observation)
            response_kuka.extend(force_values)

        # Store the observation
        # observation = {'action': np.array(action_observation.position, dtype=np.float32), 'obs': np.array(response_kuka, dtype=np.float32)}
        observation = [np.array(response_kuka, dtype=np.float32)]
        obs_deque.append(observation)

    return obs_deque


def moving_robot_cartesian_ptp_obs_no_force(action, obs_deque, kb, ati_client, box_observation):

    action_TxyzQwxyz = np.apply_along_axis(rma.TxyzRxyz_2_TxyzQwxyz, 1, action)
    action_robot = np.apply_along_axis(rosm.robodk_2_ros, 1, action_TxyzQwxyz)

    for cartesian_values in action_robot:

        ############################################################
        ####### change cartesian to joint value

        cjs = kb.get_current_joint_state()
        pose = rosm.TxyzQxyzw_2_Pose(cartesian_values)
        joint_values = kb.get_best_ik(current_joint_state=cjs, target_pose=pose, attempts=10)

        move_client_ptp(kb, joint_values.position)
        response_kuka = []
        action_observation = kb.get_current_joint_state()

        ############################################################33
        ### CHANGE THE VALUE To FK
        joint_msg_values = rosm.joint_list_2_state(joint_positions=action_observation.position, joint_names=action_observation.name)
        _fk = kb.get_fk(joint_state=joint_msg_values, header_frame_id=kb.base_)
        _Txyz_Qwxyz = [_fk.position.x, _fk.position.y, _fk.position.z, _fk.orientation.w, _fk.orientation.x, _fk.orientation.y, _fk.orientation.z]
        _Txyz_Rxyz = rma.TxyzQwxyz_2_TxyzRxyz(_Txyz_Qwxyz)


        # Get the force torque data
        force_observations = ati_client.get_force_torque()
        rclpy.spin_until_future_complete(ati_client, force_observations)

        if _Txyz_Rxyz is not None and force_observations.done():
            response_kuka = _Txyz_Rxyz
            response_ati = force_observations.result().msg
            force_values = [response_ati.fx, response_ati.fy, response_ati.fz, response_ati.tx, response_ati.ty, response_ati.tz]
            response_kuka.extend(box_observation)
            # response_kuka.extend(force_values)

        # Store the observation
        # observation = {'action': np.array(action_observation.position, dtype=np.float32), 'obs': np.array(response_kuka, dtype=np.float32)}
        observation = [np.array(response_kuka, dtype=np.float32)]
        obs_deque.append(observation)

    return obs_deque



# Example Usage
if __name__ == "__main__":
    rclpy.init()
    torch.cuda.empty_cache()
    torch.cuda.synchronize()
    ############################################################33
    checkpoint_path = '/home/cam/Downloads/checkpoints/checkpoint_no_vision_fk_model_basic_2_8_epoch_199.pth' 
    # size = (96, 96)
    box_observation = [-1.4755430707500274, -0.9344696374019131, -0.860017640151819, 1.4217170630587308, 2.1629726402745475, 0.6962921556183886, 1.9685024176693053]


    MOTION_TOLERANCE = 0.00005
    kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
                                  move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    
    # kb2 = MoveitInterface(node_name=f"obs_real_kuka_blue",     
    #                               move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
    #                               remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
    #                               prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                              )
    
    # Create an ActionClient for the ATI Force/Torque sensor
    ati_client = AtiClient()

    checkpoint = torch.load(checkpoint_path)

    # Parameters corrsponding to
    num_epochs =checkpoint['num_epochs']
    obs_dim = checkpoint['obs_dim']
    action_dim = checkpoint['action_dim']
    # parameters
    pred_horizon = checkpoint['pred_horizon']
    obs_horizon = checkpoint['obs_horizon']
    action_horizon = checkpoint['action_horizon']

    statistics = checkpoint['dataset_stats']
    len_dataloader = checkpoint['len_dataloader']   
    num_diffusion_iters = 100
    
    noise_pred_net, noise_scheduler, device, ema, optimizer, lr_scheduler = _model_initalization(action_dim, obs_dim, obs_horizon, pred_horizon, num_epochs, len_dataloader, num_diffusion_iters)
    
    noise_pred_net.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    lr_scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
    ema.load_state_dict(checkpoint['ema_state_dict'])

    # action_generator = get_robot_next_actions()
    action_obs = None
    while action_obs is None:
        action_obs = kb.get_current_joint_state()
        if action_obs is None:
            print("Warning: Failed to fetch current joint state, retrying...")
            time.sleep(0.015)  # Wait for a short time before retrying


    joint_values = rosm.joint_list_2_state(joint_positions=action_obs.position, joint_names=action_obs.name)
    _fk = kb.get_fk(joint_state=joint_values, header_frame_id=kb.base_)
    _Txyz_Qwxyz = [_fk.position.x, _fk.position.y, _fk.position.z, _fk.orientation.w, _fk.orientation.x, _fk.orientation.y, _fk.orientation.z]
    _Txyz_Rxyz = rma.TxyzQwxyz_2_TxyzRxyz(_Txyz_Qwxyz)

    box_joint_values = rosm.joint_list_2_state(joint_positions=box_observation, joint_names=action_obs.name)
    box_fk = kb.get_fk(joint_state=box_joint_values, header_frame_id=kb.base_)
    box_Txyz_Qwxyz = [box_fk.position.x, box_fk.position.y, box_fk.position.z, box_fk.orientation.w, box_fk.orientation.x, box_fk.orientation.y, box_fk.orientation.z]
    box_Txyz_Rxyz = rma.TxyzQwxyz_2_TxyzRxyz(box_Txyz_Qwxyz)

    ############################################################################
    ############################################################################
    ############################################################################
    ############# Change action to FK or joint values ##########################

    ########## fk
    response_kuka = _Txyz_Rxyz.copy()
    response_kuka.extend(box_Txyz_Rxyz)

    ########## joint
    # response_kuka = action_obs.position
    # if kb: move_client_ptp(kb, KB_HOME)

    # Get the force torque data
    print("Getting force torque data...")
    force_observations = ati_client.get_force_torque()
    rclpy.spin_until_future_complete(ati_client, force_observations)
    response_ati = force_observations.result().msg
    # print(response_ati)
    # ati_client.get_logger().info(f"Ati Service call successful {response_ati}")
    force_values = [response_ati.fx, response_ati.fy, response_ati.fz, response_ati.tx, response_ati.ty, response_ati.tz]
    response_kuka.extend(force_values)

    # observation = {'action': np.array(action_observation.position, dtype=np.float32), 'obs': np.array(response_kuka, dtype=np.float32)}
    observation = np.array(response_kuka, dtype=np.float32)

    obs_deque = collections.deque(
    [observation] * obs_horizon, maxlen=obs_horizon)
    
    action =  predicted_action(obs_deque,  statistics, obs_horizon,
                            pred_horizon, action_horizon, action_dim, noise_scheduler, num_diffusion_iters,
                            noise_pred_net, device)
    
    # plt.ion()
    # fig = plt.figure()
    # ax = plt.gca()
    # ax.legend(['J7'])
    # plt.show()

    while True:
        # obs_deque = move_robot_get_obs(action, obs_deque, kb, kb2, pipeline, ati_client)
        # obs_deque = moving_robot_joint_ptp_obs(action, obs_deque, kb, ati_client, box_Txyz_Rxyz)
        obs_deque = moving_robot_cartesian_ptp_obs(action, obs_deque, kb, ati_client, box_Txyz_Rxyz)
        # obs_deque = moving_robot_cartesian_ptp_obs_no_force(action, obs_deque, kb, ati_client, box_Txyz_Rxyz)
        action =  predicted_action(obs_deque,  statistics, obs_horizon,
                            pred_horizon, action_horizon, action_dim, noise_scheduler, num_diffusion_iters,
                            noise_pred_net, device)
    
        # plot the actions which is a list of 7 values for each joint. Also add the legend
        # ax.plot(action[0])


    
