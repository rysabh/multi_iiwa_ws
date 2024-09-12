from moveit_motion.diffusion_policy_cam.submodules import robomath as rm
from moveit_motion.diffusion_policy_cam.submodules import robomath_addon as rma
from moveit_motion.ros_submodules import ros_math as rosm
import numpy as np
from math import pi
import re
import moveit_motion.ros_submodules.ros_math as rosm
from itertools import combinations

list_xyz_mocap = np.array(
    [
        [715.696655, -422.214203, 945.778137],
        [ 709.424255, -422.107483, 973.316711],
        [630.185608, -424.013428, 806.596313],
        [472.64566, -429.681396, 862.790466],
        [514.387085, -428.120728, 818.618042]
    ]
)


list_xyz_robodk = np.apply_along_axis(rma.motive_2_robodk_marker, 1, list_xyz_mocap)

# take mean of the points
mean_xyz_robodk = np.mean(list_xyz_robodk, axis=0)

print("mean_xyz_robodk: \n", mean_xyz_robodk)
combs = np.array(list(combinations(list_xyz_robodk, 3)))

# print("combs: \n", combs)


import numpy as np
def circumcenter_and_normal_vector(points: np.ndarray) -> tuple:
    if points.shape[0] != 3 or points.shape[1] != 3:
        raise ValueError("Input must be a 3x3 array where each row represents a point in 3D space.")
    
    # Extracting individual points
    p1, p2, p3 = points[0], points[1], points[2]
    
    # Calculate vectors from p1 to p2 and p1 to p3
    v1 = p2 - p1
    v2 = p3 - p1
    
    # Normal vector to the plane defined by the three points
    normal_vector = np.cross(v1, v2)
    
    # Perpendicular bisectors in 3D
    mid1 = (p1 + p2) / 2
    mid2 = (p1 + p3) / 2
    mid3 = (p2 + p3) / 2
    
    perp_bisector_dir1 = np.cross(normal_vector, v1)
    perp_bisector_dir2 = np.cross(normal_vector, v2)
    
    # Equations of the perpendicular bisectors from midpoints
    # They are in the format A*x = b, where A = [[a1, b1, c1], [a2, b2, c2], [a3, b3, c3]]
    # x is the circumcenter (cx, cy, cz)
    # b = [dot(A[0], mid1), dot(A[1], mid2), dot(A[2], mid3)]
    A = np.array([perp_bisector_dir1, perp_bisector_dir2, normal_vector])
    b = np.array([np.dot(perp_bisector_dir1, mid1),
                  np.dot(perp_bisector_dir2, mid2),
                  np.dot(normal_vector, mid1)])  # Using any of the points

    # Solve the linear system to find the circumcenter
    circumcenter = np.linalg.solve(A, b)

    # Ensure the normal vector is a unit vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)

    return circumcenter, normal_vector

# Example usage
# points = np.array([[1, 0, 0], [4, 0, 0], [2.5, 3, 0]])
# center, normal = circumcenter_and_normal_vector(points)
# print("Circumcenter:", center)
# print("Normal Vector:", normal)


# print("combs: \n", combs)

centers_plane = [circumcenter_and_normal_vector(points) for points in combs]
# centers_plane = centers_plane[0:-1]
mean_center = np.mean([center for center, _ in centers_plane], axis=0)
mean_normal = np.mean([normal for _, normal in centers_plane], axis=0)

print("mean_center: \n", mean_center)
print("mean_normal: \n", mean_normal)

