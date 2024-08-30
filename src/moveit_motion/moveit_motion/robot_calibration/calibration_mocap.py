from moveit_motion.diffusion_policy_cam.submodules import robomath as rm
from moveit_motion.diffusion_policy_cam.submodules import robomath_addon as rma
from moveit_motion.ros_submodules import ros_math as rosm
import numpy as np

def minimize_error(world_points, robot_points):
    assert len(world_points) == len(robot_points), "Point sets must be of the same length"
    
    W_T_R_list = []
    for W_TxyzQwxyz_P, R_TxyzRxyz_P in zip(world_points, robot_points):
        # Convert the pose representations
        R_T_P = rm.KUKA_2_Pose(R_TxyzRxyz_P)
        W_T_P = rma.TxyzQwxyz_2_Pose(W_TxyzQwxyz_P)
        
        # Compute the transformation from world to robot frame
        W_T_R = W_T_P * rm.invH(R_T_P)
        W_T_R_list.append(W_T_R)
    
    # Average the transformations (in practice, consider using SVD for rotation averaging)
    avg_W_T_R = np.mean(W_T_R_list, axis=0)

    return avg_W_T_R

# Example usage with multiple points
world_points = [
    [1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.4],
    [2.0, 3.0, 4.0, 0.2, 0.3, 0.4, 0.5],
    # Add more points
]

robot_points = [
    [0.5, 1.5, 2.5, 0.05, 0.15, 0.25],
    [1.5, 2.5, 3.5, 0.15, 0.25, 0.35],
    # Add more points
]

avg_transformation = minimize_error(world_points, robot_points)
print("Average Transformation Matrix:")
print(avg_transformation)

