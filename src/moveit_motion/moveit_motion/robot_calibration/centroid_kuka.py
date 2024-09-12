from moveit_motion.diffusion_policy_cam.submodules import robomath as rm
from moveit_motion.diffusion_policy_cam.submodules import robomath_addon as rma
from moveit_motion.ros_submodules import ros_math as rosm
import numpy as np
from math import pi
import re
import moveit_motion.ros_submodules.ros_math as rosm

def parse_pose_data(pose_data):
    # Regex to find all floating-point numbers including negatives
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", pose_data)
    
    # Convert the strings to float
    numbers = [float(num) for num in numbers]
    
    # Reorder the values to [X, Y, Z, w, x, y, z]
    if len(numbers) == 7:
        return [numbers[0], numbers[1], numbers[2], numbers[6], numbers[3], numbers[4], numbers[5]]
    else:
        return "Error: Incorrect number of values found in the input string."
    
base_data = """
position:
    x: -0.4159572720527649
    y: -0.2617773413658142
    z: 0.5754642486572266
orientation:
    x: -0.0011496678926050663
    y: 0.36257627606391907
    z: 0.0029605869203805923
    w: 0.9319486618041992
"""

base_data = parse_pose_data(base_data)
base_TxyzQwxyz = base_data
base_TxyzQwxyz =  rma.motive_2_robodk_rigidbody(base_TxyzQwxyz)
base_TxyzQwxyz = [ i*1000 for i in base_TxyzQwxyz[:3]] +  base_TxyzQwxyz[3:]
print("base_TxyzQwxyz: \n", [round(i, 7) for i in base_TxyzQwxyz])
base_TxyzRxyz = rm.Pose_2_KUKA(rma.TxyzQwxyz_2_Pose(base_TxyzQwxyz))
base_TxyzRxyz = base_TxyzRxyz[:3] + [ i*pi/180 for i in base_TxyzRxyz[3:]]
print("base_TxyzRxyz: \n", [round(i, 7) for i in base_TxyzRxyz])


"""
position:
    x: -0.4188297390937805
    y: -0.262932687997818
    z: 0.5725014805793762
orientation:
    x: 0.0004088410350959748
    y: 0.3596576154232025
    z: 0.005029148887842894
    w: 0.933070719242096
"""