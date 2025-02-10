# GNU GENERAL PUBLIC LICENSE
# Version 3, 29 June 2007
# 
# Copyright (C) 2024 Rishabh Shukla
# email: rysabh@gmail.com
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# see <https://www.gnu.org/licenses/>.
# 
# Written by Rishabh Shukla

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
        # print("R_T_P: ", R_T_P)
        W_T_P = rma.TxyzQwxyz_2_Pose(W_TxyzQwxyz_P)
        # print("W_T_P: ", W_T_P)
        
        # Compute the transformation from world to robot frame
        W_T_R = W_T_P * rm.invH(R_T_P)
        # print("W_T_R: ", W_T_R)
        W_T_R_list.append(W_T_R)
        
    
    # Average the transformations (in practice, consider using SVD for rotation averaging)
    # avg_W_T_R = np.mean(W_T_R_list, axis=0)

    return W_T_R_list

# Example usage with multiple points

world_points = [
    [1172.231812, 57.700745, 175.775574, 0.529326, -0.836879, 0.069937, -0.120649], #mocap frame
    # [2.0, 3.0, 4.0, 0.2, 0.3, 0.4, 0.5],
    # Add more points
]


# write code for running rma.motive_2_robodk_rigidbody for all points in world_points
for i in range(len(world_points)):
    world_points[i] = rma.motive_2_robodk_rigidbody(world_points[i])


robot_points = [
    [0.65035, -0.31351, 0.52362, 0.11093, 0.06069, 0.15004],
    # [1.5, 2.5, 3.5, 0.15, 0.25, 0.35],
    # Add more points
]



avg_transformation = minimize_error(world_points, robot_points)
# print("Average Transformation Matrix:")

# print(avg_transformation)

print("Average Transformation Matrix (as a list):")

out = []

# write code for running rma.motive_2_robodk_rigidbody for all points in world_points
for i in range(len(avg_transformation)):
    out.append(rm.pose_2_xyzrpw(avg_transformation[i]))

# out = rma.TxyzRxyz_2_TxyzQwxyz(out[0])
# out = rma.robodk_2_motive(out)
# out = rma.TxyzQwxyz_2_TxyzRxyz(out)



print(out)


