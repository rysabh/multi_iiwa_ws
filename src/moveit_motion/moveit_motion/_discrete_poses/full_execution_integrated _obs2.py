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
# from moveit_motion.ros_submodules.RobotInterface import RobotInterface
import moveit_motion.ros_submodules.RS_submodules as rsmod
from concurrent.futures import ThreadPoolExecutor, as_completed

from diffusion_interface.srv import DiffusionAction  # Adjust this import according to your service definition
from diffusion_interface.msg import Observation
# from mocap_optitrack_interfaces.srv import GetMotionCaptureData
from diffusion_service.diffusion_inference_client import DiffusionClient



def move_client_ptp(_client, goal_list: list, tolerance=0.0005, time_out=60):
    _cjs = _client.get_current_joint_state()
    _goal_state = rosm.joint_list_2_state(goal_list, _cjs.name)

    if rsmod.MSE_joint_states(_cjs, _goal_state) < tolerance:
        print("Already at goal")
        return
    
    _goal_plan_handle = _client.get_joint_ptp_plan(_cjs, _goal_state, max_velocity_scaling_factor=0.1)
    
    if input(f"Execute {_client.move_group_name_} plan? (y/n): ").strip().lower() == 'y':
        _client.execute_joint_traj(_goal_plan_handle['trajectory'])

        _tick = time.time()
        execution_finished = False
        print("Waiting for execution to finish...")
        while not execution_finished:
            mse_client = get_mse_planend_current(_client, _goal_plan_handle)

            if (mse_client < 0.0002):
                execution_finished = True
            
            _tock = time.time()

            if _tock - _tick > time_out: print(f"Timeout {time_out} seconds: Execution not finished"); break
            time.sleep(0.01)

def plan_client_cartesian(_client, waypoints: list, max_motion_threshold= float, max_attemps: int = 5):
    for _attempt in range(max_attemps):
        _cartesian_plan_handle = _client.get_cartesian_spline_plan(
            waypoints=waypoints, planning_frame='world',
            attempts=1,
            _planner_type="cartesian_sequence_action", 
            allowed_planning_time=10.0, max_velocity_scaling_factor=0.95,
            max_acceleration_scaling_factor=0.1, num_planning_attempts=100
        )

        print("\n\n\=====================================================")
        print(_client.spline_client_._action_name.split("/")[-1])
        print("=====================================================\n\n\n")

        if _client.spline_client_._action_name.split("/")[-1] == "sequence_move_group":
            return
        
        _start_traj_point = _cartesian_plan_handle['trajectory'].joint_trajectory.points[0]
        _end_traj_point = _cartesian_plan_handle['trajectory'].joint_trajectory.points[-1]
        _mse = rsmod.MSE_joint_points(_start_traj_point, _end_traj_point)
        print(f"{_client.move_group_name_} ->Attempt: {_attempt+1} -> MSE: {round(_mse,4)}")

        if _mse <= max_motion_threshold:
            return _cartesian_plan_handle
        print(f"Retrying...")
    print(f"Max attempts reached. Could not achieve MSE <= {max_motion_threshold}. Exiting...")
    return None

# def get_mse_planend_current(_client, plan_handle):
#     _current_joint_state = _client.get_current_joint_state()
#     _target_joint_state = copy.deepcopy(_current_joint_state)
#     _target_joint_state.position = plan_handle['trajectory'].joint_trajectory.points[-1].positions
#     return rsmod.MSE_joint_states(_current_joint_state, _target_joint_state)

def get_mse_planend_current(_client, _trajectory):
    _current_joint_state = _client.get_current_joint_state()
    _target_joint_state = copy.deepcopy(_current_joint_state)
    _target_joint_state.position = _trajectory.joint_trajectory.points[-1].positions
    return rsmod.MSE_joint_states(_current_joint_state, _target_joint_state)


KG_HOME = [-0.614, 0.634, 2.302, -1.634, 1.526, -1.549, -1.897]
KB_HOME = [-0.590, -1.136, -2.251, 1.250, -1.929, 0.964, 0.494]


KG_CHISEL_START = [-0.908, 1.000, 2.218, -1.330, 1.377, -1.391, -2.146]
KB_GRIPPER_START = [-0.548, -0.289, -1.942, 1.609, -1.596, 1.258, -0.877]
###############################
#------------ Main -----------#
###############################
def main():
    rclpy.init()
    ###############################
    #------ Create Clients -------#
    ###############################

    kg =None; kb = None    
    kg = MoveitInterface(node_name=f"client_real_kuka_green",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_green",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )


    # kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
    #                               move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
    #                               remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
    #                               prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                              )

    # kg = MoveitInterface(node_name=f"client_sim_kuka_green",     
    #                                 move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
    #                                 remapping_name="",           # lbr # ""          #-> required for service and action remapping
    #                                 prefix="kuka_green",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                                 )


    # kb = MoveitInterface(node_name=f"client_sim_kuka_blue",     
    #                             move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
    #                             remapping_name="",           # lbr # ""          #-> required for service and action remapping
    #                             prefix="kuka_blue",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                             )
    
    diffusion_client = DiffusionClient()



    #-----------------------------#
    
    # action_generator = get_robot_next_actions()

    if kg: move_client_ptp(kg, KG_HOME)
    if kb: move_client_ptp(kb, KB_HOME)
    
    if kg: move_client_ptp(kg, KG_CHISEL_START)
    if kb: move_client_ptp(kb, KB_GRIPPER_START)
    
    try :
        observations_1 = None
        while True:
            _data_points_chisel = []
            _data_points_gripper = []
            
            time.sleep(0.01)
            observations_2 = diffusion_client.get_mocap_data()
            rclpy.spin_until_future_complete(diffusion_client, observations_2)

            if observations_2.done():
                # Wait until the mocap service call completes
                response_mocap = observations_2.result()
                diffusion_client.get_logger().info(f'Mocap Service call completed')

                # Send the observation to the diffusion service
                new_test = []
                new_ob_1 = Observation()
                new_ob_2 = Observation()
                new_ob_2.capture_data =  response_mocap.latest_message
                # print(response_mocap.latest_message)
                if observations_1 is None:
                    new_test.append(new_ob_2)
                    new_test.append(new_ob_2)
                else:
                    # new_test.append(new_ob)
                    new_ob_1.capture_data = observations_1.result().latest_message
                    new_test.append(new_ob_1)
                    new_test.append(new_ob_2)
                # new_test.append(new_ob)
                print("New test: ", len(new_test))
                # print("new_test")
                print("############################################")
                actions = diffusion_client.send_observation(new_test)

                # Wait until the diffusion service call completes
                rclpy.spin_until_future_complete(diffusion_client, actions)
                
                if actions.done():
                    # diffusion_client.get_logger().info(f'Response: {actions.result().actions}')
                    # print("Received data: ", actions.result().actions)
                    for value in actions.result().actions:
                        _data_points_chisel.append(value.data[0:6])
                        _data_points_gripper.append(value.data[6:12])
            
            
            
            print("Gripper: ", _data_points_gripper)
            
            print("Chisel: ", _data_points_chisel)
            
        
            # _data_points_chisel = next_actions['data_chisel'] 
            _data_points_chisel = np.apply_along_axis(rma.TxyzRxyz_2_TxyzQwxyz, 1,_data_points_chisel)
            _data_points_chisel = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_chisel)
            _pose_waypoints_chisel = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_chisel)
            _pose_waypoints_chisel = _pose_waypoints_chisel.tolist()

            # _data_points_gripper = next_actions['data_gripper']
            _data_points_gripper = np.apply_along_axis(rma.TxyzRxyz_2_TxyzQwxyz, 1,_data_points_gripper)
            _data_points_gripper = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_gripper)
            _pose_waypoints_gripper = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_gripper)
            _pose_waypoints_gripper = _pose_waypoints_gripper.tolist()

            # CARTESIAN_MSE_THRESHOLD = 0.0002
            CARTESIAN_MSE_THRESHOLD = 1
            
            if kg: kg_plan_handle = plan_client_cartesian(kg, _pose_waypoints_chisel, CARTESIAN_MSE_THRESHOLD, 5)
            if kb: kb_plan_handle = plan_client_cartesian(kb, _pose_waypoints_gripper, CARTESIAN_MSE_THRESHOLD, 5)


            EXECUTE_FLAG = 'y' #input("Execute trajectory? (y/n): ").strip().lower()
            
            if EXECUTE_FLAG == 'y':
                if kg: kg.execute_joint_traj(kg_plan_handle['trajectory'])
                if kb: kb.execute_joint_traj(kb_plan_handle['trajectory'])

                _tick = time.time()
                execution_finished = False
                while not execution_finished:
                    mse_kg = 0
                    if kg: mse_kg = get_mse_planend_current(kg, kg_plan_handle['trajectory'])

                    mse_kb = 0
                    if kb: mse_kb = get_mse_planend_current(kb, kb_plan_handle['trajectory'])

                    if (mse_kg < 0.0002) and (mse_kb < 0.0002):
                        execution_finished = True
                    
                    _tock = time.time()
                    if _tock - _tick > 10: print("Timeout: Execution not finished"); break

                    time.sleep(0.01)
                    
                    
                observations_1 = observations_2
                
                    
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting loop...")


    print("Remaining code executed after the loop.")
    if kg: move_client_ptp(kg, KG_HOME)
    if kb: move_client_ptp(kb, KB_HOME)
    rclpy.shutdown()

if __name__ == '__main__':
    import sys

    # _file_name = sys.argv[1] if len(sys.argv) > 1 else "ft_010.csv"

    main()
