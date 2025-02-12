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

import numpy as np

def horn_method(A, B):
    """
    Perform the Horn's method for absolute orientation.
    
    A: Nx3 numpy array of 3D points in the world frame
    B: Nx3 numpy array of corresponding 3D points in the robot frame
    
    Returns:
    R: 3x3 rotation matrix
    t: 3x1 translation vector
    """
    
    assert A.shape == B.shape, "Input point clouds must have the same dimensions"
    
    # Number of points
    n = A.shape[0]
    
    # Compute centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # Center the points
    AA = A - centroid_A
    BB = B - centroid_B
    
    # Compute the covariance matrix
    H = AA.T @ BB
    
    # Perform Singular Value Decomposition (SVD)
    U, S, Vt = np.linalg.svd(H)
    
    # Compute rotation
    R = Vt.T @ U.T
    
    # Special reflection case
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    
    # Compute translation
    t = centroid_B.T - R @ centroid_A.T
    
    return R, t

# Example usage
world_points = np.array([
    [1.0, 2.0, 3.0],
    [2.0, 3.0, 4.0],
    [3.0, 4.0, 5.0],
    # Add more points
])

robot_points = np.array([
    [0.5, 1.5, 2.5],
    [1.5, 2.5, 3.5],
    [2.5, 3.5, 4.5],
    # Add more points
])

R, t = horn_method(world_points, robot_points)

print("Rotation Matrix R:")
print(R)
print("\nTranslation Vector t:")
print(t)
