from moveit_motion.diffusion_policy_cam.submodules import robomath as rm
from moveit_motion.diffusion_policy_cam.submodules import robomath_addon as rma
from moveit_motion.ros_submodules import ros_math as rosm
import numpy as np

kg_base_robodk_TxyzRxyz = [0.8, 0.7, -0.4, 0, 0, -2.356]
kg_base_robodk_TxyzQwxyz = rma.TxyzRxyz_2_TxyzQwxyz(kg_base_robodk_TxyzRxyz)
kg_base_motive_TxyzQwxyz = rma.robodk_2_motive(kg_base_robodk_TxyzQwxyz)

kb_base_robodk_TxyzRxyz = [0.55, -0.4, -0.25, 0, 0, 0.783]
kb_base_robodk_TxyzQwxyz = rma.TxyzRxyz_2_TxyzQwxyz(kb_base_robodk_TxyzRxyz)
kb_base_motive_TxyzQwxyz = rma.robodk_2_motive(kb_base_robodk_TxyzQwxyz)

print("kg_base_motive_TxyzQwxyz: \n", kg_base_motive_TxyzQwxyz)
print("----------------------------------------")
print("kb_base_motive_TxyzQwxyz: \n", kb_base_motive_TxyzQwxyz)

