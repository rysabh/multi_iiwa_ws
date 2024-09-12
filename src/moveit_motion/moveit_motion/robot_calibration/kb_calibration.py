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
    
# kg_base_robodk_TxyzRxyz = [0.8, 0.7, -0.4, 0, 0, -2.356]
# kg_base_robodk_TxyzQwxyz = rma.TxyzRxyz_2_TxyzQwxyz(kg_base_robodk_TxyzRxyz)
# kg_base_motive_TxyzQwxyz = rma.robodk_2_motive(kg_base_robodk_TxyzQwxyz)

# kb_base_robodk_TxyzRxyz = [0.55, -0.4, -0.25, 0, 0, 0.783]
# kb_base_robodk_TxyzQwxyz = rma.TxyzRxyz_2_TxyzQwxyz(kb_base_robodk_TxyzRxyz)
# kb_base_motive_TxyzQwxyz = rma.robodk_2_motive(kb_base_robodk_TxyzQwxyz)

# print("kg_base_motive_TxyzQwxyz: \n", kg_base_motive_TxyzQwxyz)
# print("----------------------------------------")
# print("kb_base_motive_TxyzQwxyz: \n", kb_base_motive_TxyzQwxyz)

############################################################################################################

# W_TxyzQwxyz_B = [-0.4125173091888428, -0.266859233379364, 0.5796912312507629, 0.9316409826278687, -0.0010835565626621246, 0.363003134727478, 0.016511525958776474]
# W_TxyzQwxyz_B = rma.motive_2_robodk_rigidbody(W_TxyzQwxyz_B)

# # W_TxyzRxyz_B = rm.Pose_2_KUKA(rma.TxyzQwxyz_2_Pose(W_TxyzQwxyz_B))
# W_TxyzRxyz_B = rma.TxyzQwxyz_2_TxyzRxyz(W_TxyzQwxyz_B)

# # W_TxyzRxyz_B = [ i*1000 for i in W_TxyzRxyz_B[:3]] + [ i*pi/180 for i in W_TxyzRxyz_B[3:]]

# W_TxyzRxyz_B = [ i*1000 for i in W_TxyzRxyz_B[:3]] +  W_TxyzRxyz_B[3:]

# print("W_TxyzRxyz_B: \n", W_TxyzRxyz_B)


############################################################################################################
'''
print("-----------------------------------------------------------------------------------------------------------------")
base_data = """
position:
    x: 0.17086389660835266
    y: 0.0008673427510075271
    z: 0.23214586079120636
orientation:
    x: -0.0010002314811572433
    y: -0.6588081121444702
    z: 0.0029677206184715033
    w: 0.7523044943809509
"""

      


base_data = parse_pose_data(base_data)
# print("base_data: \n", [round(i, 7) for i in base_data])
base_TxyzQwxyz = base_data
base_TxyzQwxyz =  rma.motive_2_robodk_rigidbody(base_TxyzQwxyz)
base_TxyzQwxyz = [ i*1000 for i in base_TxyzQwxyz[:3]] +  base_TxyzQwxyz[3:] # meters

# base_TxyzQwxyz = rosm.robodk_2_ros(base_TxyzQwxyz)
print("base_TxyzQwxyz: \n", [round(i, 7) for i in base_TxyzQwxyz])


# base_TxyzRxyz = rma.TxyzQwxyz_2_TxyzRxyz(base_TxyzQwxyz) #meters and radians #don't use this for KUKA calibration; this is just for sanity check that rviz and this should match exactly
base_TxyzRxyz = rm.Pose_2_KUKA(rma.TxyzQwxyz_2_Pose(base_TxyzQwxyz)); #meter and degrees
# base_TxyzRxyz = base_TxyzRxyz[:3] + [ i*pi/180 for i in base_TxyzRxyz[3:]] # meters and radians
# base_TxyzRxyz = base_TxyzRxyz[:3] + [ i*180/pi for i in base_TxyzRxyz[3:]]
print("base_TxyzRxyz: \n", [round(i, 7) for i in base_TxyzRxyz])



base_data_rviz = [0.23222, 0.17079, 0.0005317, 0.75224, 0.0025645, -0.00068819, -0.65888] # meters and radians

# base_TxyzRxyz_rviz = rma.TxyzQwxyz_2_TxyzRxyz(base_data_rviz)
# base_TxyzRxyz_rviz = base_TxyzRxyz_rviz[:3] + [ i*180/pi for i in base_TxyzRxyz_rviz[3:]] # meters and degrees
base_TxyzRxyz_rviz = rm.Pose_2_KUKA(rma.TxyzQwxyz_2_Pose(base_data_rviz)) # meters and degrees
print("base_TxyzRxyz_rviz: \n", [round(i, 7) for i in base_TxyzRxyz_rviz])

# subtract base_TxyzRxyz from base_TxyzRxyz_rviz
base_TxyzRxyz_diff = [base_TxyzRxyz[i] - base_TxyzRxyz_rviz[i] for i in range(6)]; base_TxyzRxyz_diff = [i*1000 for i in base_TxyzRxyz_diff[0:3]] + base_TxyzRxyz_diff[3:]
print("-----------------------------------------------------------------------------------------------------------------")
print("Error in Base Motive vs ROS (mm + deg): \n", [round(i, 7) for i in base_TxyzRxyz_diff])
'''


############################################################################################################
print("==============================================================================================================")

tool_data = """
position:
    x: 0.1498989760875702
    y: 1.2517316341400146
    z: 0.22835218906402588
orientation:
    x: -0.00013029460387770087
    y: 0.7515493035316467
    z: -0.000042
    w: 0.6596769690513611
"""

tool_data = parse_pose_data(tool_data) 
print("pose_data: \n", [round(i, 7) for i in tool_data])

Tool_W_TxyzQwxyz = tool_data
Tool_W_TxyzQwxyz =  rma.motive_2_robodk_rigidbody(Tool_W_TxyzQwxyz)
Tool_W_TxyzQwxyz = [ i*1000 for i in Tool_W_TxyzQwxyz[:3]] +  Tool_W_TxyzQwxyz[3:] #in meters radians
# Tool_W_TxyzQwxyz = rosm.robodk_2_ros(Tool_W_TxyzQwxyz)
print("Tool_W_TxyzQwxyz: \n", [round(i, 7) for i in Tool_W_TxyzQwxyz]) # in meters and radians

# Tool_W_TxyzRxyz = rma.TxyzQwxyz_2_TxyzRxyz(Tool_W_TxyzQwxyz); Tool_W_TxyzRxyz = Tool_W_TxyzRxyz[:3] + [ i*180/pi for i in Tool_W_TxyzRxyz[3:]]
Tool_W_TxyzRxyz = rm.Pose_2_KUKA(rma.TxyzQwxyz_2_Pose(Tool_W_TxyzQwxyz)) #in meters and degrees
print("Tool_W_TxyzRxyz: \n", [round(i, 4) for i in Tool_W_TxyzRxyz])


Tool_W_TxyzQwxyz_rviz = [0.22757, 0.15945, 1.2509, 0.6595, -0.00072855, -0.0026592, 0.7517]

print("Tool_W_TxyzQwxyz_rviz: \n", Tool_W_TxyzQwxyz_rviz)
# Tool_W_TxyzRxyz_rviz = rma.TxyzQwxyz_2_TxyzRxyz(Tool_W_TxyzQwxyz_rviz); Tool_W_TxyzRxyz_rviz = Tool_W_TxyzRxyz_rviz[:3] + [ i*180/pi for i in Tool_W_TxyzRxyz_rviz[3:]]
Tool_W_TxyzRxyz_rviz = rm.Pose_2_KUKA(rma.TxyzQwxyz_2_Pose(Tool_W_TxyzQwxyz_rviz)) #in meters and degrees

print("Tool_W_TxyzRxyz_rviz: \n", [round(i, 4) for i in Tool_W_TxyzRxyz_rviz])

print("-----------------------------------------------------------------------------------------------------------------")

# subtract Tool_W_TxyzRxyz from Tool_W_TxyzRxyz_rviz
Tool_W_TxyzRxyz_diff = [Tool_W_TxyzRxyz[i] - Tool_W_TxyzRxyz_rviz[i] for i in range(6)]; Tool_W_TxyzRxyz_diff = [i*1000 for i in Tool_W_TxyzRxyz_diff[0:3]] + Tool_W_TxyzRxyz_diff[3:]

print("Error in Tool Motive vs ROS (mm + deg): \n", [round(i, 7) for i in Tool_W_TxyzRxyz_diff])

print("==============================================================================================================")

print("**these errors could be due to either base calibration or tool calibration or both in \"motive <-> urdf\" conversion**")
