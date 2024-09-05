#NOT WORKING
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface
import copy
import moveit_motion.ros_submodules.ros_math as rosm
import numpy as np
import moveit_motion.diffusion_policy_cam.submodules.cleaned_file_parser as cfp
import moveit_motion.diffusion_policy_cam.submodules.robomath_addon as rma
import moveit_motion.diffusion_policy_cam.submodules.robomath as rm
import time
import csv
from math import pi
from moveit_motion.ros_submodules.RobotInterface import RobotInterface
import moveit_motion.ros_submodules.RS_submodules as rsmod
from concurrent.futures import ThreadPoolExecutor, as_completed

_file_name = "ft_010.csv"
path = f'no-sync/edge_3/{_file_name}'
data_loader = cfp.DataParser.from_quat_file(file_path = path, target_fps= 60, filter=False, window_size=30, polyorder=3)
#while not input is to quit
FRACTION_TO_RUN = 1.0
SLOWNESS_FACTOR = 5.0


_data_chisel = data_loader.get_rigid_TxyzQwxyz()['chisel']
_data_gripper = data_loader.get_rigid_TxyzQwxyz()['gripper']

# define int(FRACTION_TO_RUN*len(_data_chisel)
index_ = int(FRACTION_TO_RUN*len(_data_chisel))

_data_chisel = _data_chisel[:index_]
_data_gripper = _data_gripper[:index_]
_data_times = data_loader.get_time()[:index_]

print(f"Number of data points for chisel: {len(_data_chisel)}")
print(f"Number of data points for gripper: {len(_data_gripper)}")
print(f"Number of data points for time: {len(_data_times)}")


def get_robot_next_actions():
    step_size = 8
    # Zip the data_chisel and data_gripper so we can iterate over them simultaneously
    for i in range(0, len(_data_chisel), step_size):
        data_chisel_chunk = _data_chisel[i:i+step_size]
        data_gripper_chunk = _data_gripper[i:i+step_size]
        
        # Yield the chunks for both chisel and gripper
        yield {
            'data_chisel': data_chisel_chunk,
            'data_gripper': data_gripper_chunk
        }

    
def main():
    rclpy.init()
    kg =None; kb = None
    
    kg = MoveitInterface(node_name=f"client_kuka_green",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_green",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )


    kb = MoveitInterface(node_name=f"client_kuka_blue",     
                                  move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    
    # kg_cjs = kg.get_current_joint_state(); # print("kg_cjs: ", kg_cjs.position)

    try:
        while True:
            next_actions = get_robot_next_actions()
            _data_points_chisel = next_actions['data_chisel']
            _data_points_chisel = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_chisel)
            _pose_waypoints_chisel = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_chisel)
            _pose_waypoints_chisel = _pose_waypoints_chisel.tolist()

            _data_points_gripper = next_actions['data_gripper']
            _data_points_gripper = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_gripper)
            _pose_waypoints_gripper = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_gripper)
            _pose_waypoints_gripper = _pose_waypoints_gripper.tolist()


            if kg:
                kg_plan_handle = kg.get_cartesian_spline_plan(
                    waypoints=_pose_waypoints_chisel, planning_frame='world',
                    _planner_type="cartesian_interpolator", max_step=0.01,
                    jump_threshold=0.0, avoid_collisions=False, attempts=1
                )
                print(f"Fraction of path executed (kg): {kg_plan_handle['fraction']}\nStop flag (kg): {kg_plan_handle['stop_flag']}")


            if kb:
                kb_plan_handle = kb.get_cartesian_spline_plan(
                    waypoints=_pose_waypoints_gripper, planning_frame='world',
                    _planner_type="cartesian_interpolator", max_step=0.01,
                    jump_threshold=0.0, avoid_collisions=False, attempts=1
                )
                print(f"Fraction of path executed (kb): {kb_plan_handle['fraction']}\nStop flag (kb): {kb_plan_handle['stop_flag']}")


            EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()

            if EXECUTE_FLAG == 'y':
                if kg:
                    kg.execute_joint_traj(kg_plan_handle['trajectory'])
                
                if kb:
                    kb.execute_joint_traj(kb_plan_handle['trajectory'])

                # wait for both to finish, however, have a timeout

                _tick = time.time()
                while not execution_finished:
                    mse_kg = 0
                    if kg:
                        kg_cjs = kg.get_current_joint_state()
                        kg_tjs = copy.deepcopy(kg_cjs)
                        kg_tjs.position = kg_plan_handle['trajectory'].joint_trajectory.points[-1].positions
                        mse_kg = rsmod.MSE_joint_states(kg_cjs, kg_tjs)

                    mse_kb = 0
                    if kb:
                        kb_cjs = kb.get_current_joint_state()
                        kb_tjs = copy.deepcopy(kb_cjs)
                        kb_tjs.position = kb_plan_handle['trajectory'].joint_trajectory.points[-1].positions
                        mse_kb = rsmod.MSE_joint_states(kb_cjs, kb_tjs)


                    if (mse_kg < 0.0002) and (mse_kb < 0.0002):
                        execution_finished = True
                    
                    _tock = time.time()
                    if _tock - _tick > 10:
                        print("Timeout: Execution not finished")
                        break

                time.sleep(0.01)
            else:
                print("Trajectory not executed")
            
    except StopIteration:
        print("All actions have been exhausted.")

        rclpy.shutdown()

if __name__ == '__main__':
    import sys

    # _file_name = sys.argv[1] if len(sys.argv) > 1 else "ft_010.csv"

    main()




    '''dump
            # with ThreadPoolExecutor() as executor:
            #     future_exec_kg = executor.submit(kg.execute_joint_traj, kg_plan_handle['trajectory'])
            #     future_exec_kb = executor.submit(kb.execute_joint_traj, kb_plan_handle['trajectory'])
                
            #     # Ensure both have started simultaneously
            #     future_exec_kg.result()
            #     future_exec_kb.result()



    '''