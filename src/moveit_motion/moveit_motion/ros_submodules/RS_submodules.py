import os
import pickle
import csv
import numpy as np
import copy
from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
    BoundingVolume,
    OrientationConstraint,
    PositionConstraint,
)

from sensor_msgs.msg import JointState

def save_trajectory(trajectory_data, file_name, dir_path):
    '''
    trajectory_data, file_name, dir_path
    '''
    full_path = os.path.join(dir_path, file_name)
    with open(full_path, 'wb') as file:
        pickle.dump(trajectory_data, file)

def save_trajectory_to_csv(trajectory_data, file_name, dir_path):
    '''
    trajectory_data, file_name, dir_path
    '''
    full_path = os.path.join(dir_path, file_name)
    with open(full_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_from_start'] + [f'joint_{i+1}' for i in range(len(trajectory_data.points[0].positions))])
        for point in trajectory_data.points:
            writer.writerow([point.time_from_start.to_sec()] + list(point.positions))


def MSE_joint_states(joint_state_1: JointState, joint_state_2: JointState) -> float:
    # Implementation of the MSE calculation between two joint states

    joint_positions_1 = dict(zip(joint_state_1.name, joint_state_1.position))
    joint_positions_2 = dict(zip(joint_state_2.name, joint_state_2.position))

    # Sanity check: Ensure the keys (joint names) match between the two joint states
    if set(joint_positions_1.keys()) != set(joint_positions_2.keys()):
        raise ValueError("Joint names for calculating MSE Error, do not match")
    
    # Extract common joint names
    _common_joint_names = joint_positions_1.keys()
    _squared_diffs = [(joint_positions_1[name] - joint_positions_2[name]) ** 2 for name in _common_joint_names]

    _mse = np.mean(_squared_diffs)
    return _mse

def MSE_joint_points(point_1, point_2):
    # points are same as joint_trajectory.points[0]
    # Extract positions from both JointTrajectoryPoint messages
    positions_1 = np.array(point_1.positions)
    positions_2 = np.array(point_2.positions)

    # Sanity check: Ensure the two position arrays have the same length
    if positions_1.shape != positions_2.shape:
        raise ValueError("Joint positions for calculating MSE do not have the same length")

    # Calculate squared differences between corresponding joint positions
    _squared_diffs = (positions_1 - positions_2) ** 2

    # Compute the Mean Squared Error
    _mse = np.mean(_squared_diffs)
    return _mse


def MSE_poses(pose_1, pose_2):
    current_position = np.array([pose_1.position.x, pose_1.position.y, pose_1.position.z])
    target_position = np.array([pose_2.position.x, pose_2.position.y, pose_2.position.z])
    
    current_orientation = np.array([pose_1.orientation.x, pose_1.orientation.y, pose_1.orientation.z, pose_1.orientation.w])
    target_orientation = np.array([pose_2.orientation.x, pose_2.orientation.y, pose_2.orientation.z, pose_2.orientation.w])
    
    mse_position = np.mean((current_position - target_position) ** 2)
    mse_orientation = np.mean((current_orientation - target_orientation) ** 2)
    mse_total = mse_position + mse_orientation
    return mse_total


def sort_joint_state(joint_state: JointState) -> JointState:
    sorted_joints = sorted(zip(joint_state.name, joint_state.position))
    joint_state.name = [joint[0] for joint in sorted_joints]
    joint_state.position = [joint[1] for joint in sorted_joints]
    return joint_state


def filter_names_by_prefix(names: list, prefix: str) -> list:
    indices = [i for i, name in enumerate(names) if name.startswith(prefix)]
    return indices

def filter_joint_state(joint_state: JointState, prefix: str) -> JointState:
    _joint_state = copy.deepcopy(joint_state)
    indices = filter_names_by_prefix(_joint_state.name, prefix)
    _joint_state.name = [_joint_state.name[i] for i in indices]
    _joint_state.position = [_joint_state.position[i] for i in indices]
    _joint_state.velocity = [_joint_state.velocity[i] for i in indices] if _joint_state.velocity else []
    _joint_state.effort = [_joint_state.effort[i] for i in indices] if _joint_state.effort else []
    return _joint_state

def modify_names(names: list, modifier: str, add_prefix: bool) -> list: 
    if add_prefix:
        return [f"{modifier}_{name}" for name in names]
    else:
        return [name.replace(f"{modifier}_", "") for name in names]
    
def modify_joint_state(joint_state: JointState, modifier: str, frame_id: str, add_prefix:bool) -> JointState:
    _joint_state = copy.deepcopy(joint_state)
    _joint_state.name = modify_names(_joint_state.name, modifier, add_prefix)
    _joint_state.header.frame_id = frame_id
    return _joint_state

def filter_trajectory(trajectory: RobotTrajectory, prefix: str) -> RobotTrajectory:
    _trajectory = copy.deepcopy(trajectory)
    _jt = _trajectory.joint_trajectory
    indices = filter_names_by_prefix(_jt.joint_names, prefix)
    _jt.joint_names = [_jt.joint_names[i] for i in indices]
    
    for point in _jt.points:
        point.positions = [point.positions[i] for i in indices] if point.positions else []
        point.velocities = [point.velocities[i] for i in indices] if point.velocities else []
        point.accelerations = [point.accelerations[i] for i in indices] if point.accelerations else []
        point.effort = [point.effort[i] for i in indices] if point.effort else []
    return _trajectory

def modify_trajectory(trajectory: RobotTrajectory, modifier: str, frame_id: str, add_prefix: bool) -> RobotTrajectory:
    _trajectory = copy.deepcopy(trajectory)
    _jt = _trajectory.joint_trajectory
    _jt.joint_names = modify_names(_jt.joint_names, modifier, add_prefix)
    _jt.header.frame_id = frame_id

    return _trajectory

# def combine_multiple_move_group_trajectories(trajectories: list[RobotTrajectory]) -> RobotTrajectory:
#     # Combine the trajectories
#     pass


# def moveit_2_robot_trajectory(moveit_trajectory: RobotTrajectory) -> RobotTrajectory:
    
#     return moveit_trajectory


# def robot_2_moveit_trajectory(robot_trajectory: RobotTrajectory) -> RobotTrajectory:
#     return robot_trajectory


###################################################
#--------------Decorators-------------------------#
###################################################
def filter_joint_state_for_prefix(func):
    def wrapper(self, *args, **kwargs):
        joint_state = func(self, *args, **kwargs)
        return filter_joint_state(joint_state, self.prefix_)
    return wrapper

# def modify_joint_state_for_moveit(func):
#     def wrapper(self, *args, **kwargs):
#         joint_state = func(self, *args, **kwargs)
#         return modify_joint_state(joint_state, self.remapping_name_, frame_id="world", add_prefix=True)
#     return wrapper

# def modify_joint_state_for_robot(func):
#     def wrapper(self, *args, **kwargs):
#         joint_state = func(self, *args, **kwargs)
#         return modify_joint_state(joint_state, self.remapping_name_, frame_id='', add_prefix=False)
#     return wrapper

def filter_trajectory_for_prefix(func):
    def wrapper(self, *args, **kwargs):
        trajectory = func(self, *args, **kwargs)
        return filter_trajectory(trajectory, self.prefix_)
    return wrapper

# def modify_trajectory_for_moveit(func):
#     def wrapper(self, *args, **kwargs):
#         trajectory = func(self, *args, **kwargs)
#         return modify_trajectory(trajectory, self.remapping_name_, frame_id="world", add_prefix=True)
#     return wrapper

# def modify_trajectory_for_robot(func):
#     def wrapper(self, *args, **kwargs):
#         trajectory = func(self, *args, **kwargs)
#         return modify_trajectory(trajectory, self.remapping_name_, frame_id='', add_prefix=False)
#     return wrapper



def modify_joint_state_for_moveit(joint_state: JointState, modifier: str):
    return modify_joint_state(joint_state, modifier, frame_id="world", add_prefix=True)

def modify_joint_state_for_robot(joint_state: JointState, modifier: str):
    return modify_joint_state(joint_state, modifier, frame_id='', add_prefix=False)

def modify_trajectory_for_moveit(trajectory: RobotTrajectory, modifier: str):
    return modify_trajectory(trajectory, modifier, frame_id="world", add_prefix=True)

def modify_trajectory_for_robot(trajectory: RobotTrajectory, modifier: str):
    return modify_trajectory(trajectory, modifier, frame_id='', add_prefix=False)