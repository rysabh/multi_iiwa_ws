import os
import pickle
import csv
import numpy as np

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

from sensor_msgs.msg import JointState
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


def MSE_poses():
    pass


def filter_for_prefix(func):
    def wrapper(self, *args, **kwargs):
        joint_state = func(self, *args, **kwargs)
        _prefix = self.prefix_
        
        if joint_state.name:
            indices = [i for i, name in enumerate(joint_state.name) if name.startswith(_prefix)]
            joint_state.name = [joint_state.name[i] for i in indices]
            joint_state.position = [joint_state.position[i] for i in indices]
            joint_state.velocity = [joint_state.velocity[i] for i in indices] if joint_state.velocity else []
            joint_state.effort = [joint_state.effort[i] for i in indices] if joint_state.effort else []
        
        return joint_state
    return wrapper

