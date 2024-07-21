import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState

from ros_submodules.MoveitInterface import MoveitInterface

from diffusion_policy_cam.submodules import cleaned_file_parser as cfp, robomath_addon as rma, robomath as rm
# from diffusion_policy_cam.submodules import cleaned_file_parser as cfp, robomath_addon as rma, robomath as rm

import pandas as pd

def create_pose(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def robodk_2_ros(TxyzQwxyz: list) -> list:
    return [TxyzQwxyz[0], TxyzQwxyz[1], TxyzQwxyz[2], TxyzQwxyz[6], TxyzQwxyz[3], TxyzQwxyz[4], TxyzQwxyz[5]]

def main():
    rclpy.init()

    # Initialize clients
    client_green = MoveitInterface(node_name="client_green", move_group_name="kuka_green")
    cjs_green = client_green.get_current_joint_state()

    file_path = "src/moveit_motion/moveit_motion/diffusion_policy_cam/diffusion_pipline/data_chisel_task/cleaned_traj/cap_010_cleaned.csv"
    file_name = os.path.basename(file_path)
    

    data = cfp.DataParser.from_quat_file(file_path = file_path, target_fps= 10.0, filter=True, window_size=15, polyorder=3)

    W_T_KG = rma.TxyzQwxyz_2_Pose(rma.motive_2_robodk_rigidbody([0.35, 0.014, -0.5, -0.999, 0.006, -0.007, -0.008]))

    columns = [f"A{i+1}" for i in range(7)]

    green_joint_values = []

    data_chisel = data.get_rigid_TxyzQwxyz()["chisel"]
    data_times = data.get_time()


    try:
        for pose, _time  in zip(data_chisel, data_times):
            print(f"Time: {_time}")
            W_T_Ch = rma.TxyzQwxyz_2_Pose(pose)
            KG_T_Ch = rm.invH(W_T_KG)*W_T_Ch
            TxyzQwxyz_green = rma.Pose_2_TxyzQwxyz(KG_T_Ch)
            TxyzQxyzw_green = robodk_2_ros(TxyzQwxyz_green)
            pose_green = create_pose(*TxyzQxyzw_green)    
            tjs_green = client_green.get_best_ik(current_joint_state=cjs_green, target_pose=pose_green, attempts=200)
            green_joint_values.append(tjs_green.position)
            cjs_green = tjs_green
    except Exception as e:
        print(e)
    finally:
        client_green.destroy_node()
        rclpy.shutdown()


    save_dir = "src/moveit_motion/moveit_motion/diffusion_policy_cam/diffusion_pipline/kg_joint_values"
    results = pd.DataFrame(green_joint_values, columns=columns)
    new_file_path = os.path.join(save_dir, f"kg_{file_name}")
    results.to_csv(new_file_path, index=False)
    print("Data saved to joint_positions.csv")

if __name__ == "__main__":
    main()