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
import moveit_motion.diffusion_policy_cam.submodules.data_filter as dft
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
            mse_client = get_mse_planend_current(_client, _goal_plan_handle['trajectory'])

            if (mse_client < 0.0002):
                execution_finished = True
            
            _tock = time.time()

            if _tock - _tick > time_out: print(f"Timeout {time_out} seconds: Execution not finished"); break
            time.sleep(0.01)



def plan_client_cartesian(_client, waypoints: list, 
                          max_motion_threshold= float, max_attemps: int = 1,
                          **kwargs):
    planner_type = "cartesian_interpolator"
    # planner_type = "cartesian_sequence_action"
    
    '''
    motion-plan-specific parameters:
        num_planning_attempts = 10, 
        allowed_planning_time=10.0, 
        max_velocity_scaling_factor=0.1,
        max_acceleration_scaling_factor=0.1,
        pipeline_id,
        planner_id,
        goal_constraints, path_constraints
        start_state

    pilz-sequence-action-specific parameters:
        blend_radius=0.000005
    
    cartesian-interpolator-specific parameters:
        max_step = 0.01 # Decreased from default to increase waypoints
        jump_threshold = 0.0 # Disable to prevent sudden jumps
        avoid_collisions = False,
        'prismatic_jump_threshold': 0.1, 
        'revolute_jump_threshold': 0.1, -> By setting the threshold to 0.1 radians, you are allowing a maximum joint angle change of about 5.73 degrees between consecutive waypoints for any revolute joint.
    '''
    
    for _attempt in range(max_attemps):
        _cartesian_plan_handle = _client.get_cartesian_spline_plan(
            waypoints=waypoints, planning_frame='world',
            attempts=10, 
            _planner_type=planner_type,
            #motion-plan-specific parameters
            allowed_planning_time=kwargs.get('allowed_planning_time', 10.0), 
            max_velocity_scaling_factor=kwargs.get('max_velocity_scaling_factor', 0.1),
            max_acceleration_scaling_factor=kwargs.get('max_acceleration_scaling_factor', 0.1),
            num_planning_attempts=kwargs.get('num_planning_attempts', 100),
            
            #pilz-sequence-action-specific parameters
            blend_radius=kwargs.get('blend_radius', 0.000005),
            
            # cartesian-interpolator-specific parameters
            max_step = kwargs.get("max_step", 0.01),
            jump_threshold = kwargs.get("jump_threshold", 0.0),
            avoid_collisions = kwargs.get("avoid_collisions", False),
            revolute_jump_threshold = kwargs.get("revolute_jump_threshold", 0.0),
            prismatic_jump_threshold = kwargs.get("prismatic_jump_threshold", 0.0),
        )
        
        slowness_factor = kwargs.get('slowness_factor', 1)
        if slowness_factor:
            _cartesian_plan_handle['trajectory'] = rosm.slow_down_trajectory(_cartesian_plan_handle['trajectory'], slowness_factor)
            
        if planner_type == "cartesian_sequence_action":
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

EXECUTION_TIMEOUT = 60
SLOWNESS_FACTOR = 5
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


    kb = MoveitInterface(node_name=f"client_real_kuka_blue",     
                                  move_group_name="kuka_blue", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="kuka_blue",           # lbr # ""          #-> required for service and action remapping
                                  prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )

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
    if kb: move_client_ptp(kb, KB_HOME)
    if kg: move_client_ptp(kg, KG_HOME)
    
    if kb: move_client_ptp(kb, KB_GRIPPER_START)
    if kg: move_client_ptp(kg, KG_CHISEL_START)
    
    
    try :
        while True:
            _data_points_chisel = []
            _data_points_gripper = []
            
            time.sleep(0.01)
            state_observations = diffusion_client.get_mocap_data(); rclpy.spin_until_future_complete(diffusion_client, state_observations)
    
            force_observations = diffusion_client.get_force_torque(); rclpy.spin_until_future_complete(diffusion_client, force_observations)

            if state_observations.done() and force_observations.done():
                # Wait until the mocap service call completes
            # Wait until the mocap service call completes
                response_mocap = state_observations.result()
                diffusion_client.get_logger().info(f'Mocap Service call completed')

                response_ati = force_observations.result()
                diffusion_client.get_logger().info(f"Ati Service call successful")


                # Send the observation to the diffusion service
                new_test = []
                new_ob = Observation()
                # print(new_ob)
                new_ob.capture_data =  response_mocap.latest_message
                print("############################################")
                
                print("Force DATA -",response_ati.msg)
                new_ob.ati_data = response_ati.msg
                print("############################################")
                
                # new_test.append(new_ob)
                new_test.append(new_ob)
                print("New test: ", len(new_test))
                # print("New test: ", new_test)
                print("############################################")
                actions = diffusion_client.send_observation(new_test)

                # Wait until the diffusion service call completes
                rclpy.spin_until_future_complete(diffusion_client, actions)
                
                print("Actions len -",len(actions.result().actions))
                
                if actions.done():
                    # diffusion_client.get_logger().info(f'Response: {actions.result().actions}')
                    # print("Received data: ", actions.result().actions)
                    for value in actions.result().actions:
                        _data_points_chisel.append(value.data[0:6])
                        _data_points_gripper.append(value.data[6:12])
            
            
            
            # print("Gripper: ", _data_points_gripper)
            
            # print("Chisel: ", _data_points_chisel)
            
        
            # _data_points_chisel = next_actions['data_chisel'] 
            _data_points_chisel = dft.filter_outliers_verbose(_data_points_chisel, threshold=2)
            # _data_points_chisel = dft.smooth_waypoints_sg(_data_points_chisel, window_length=4, polyorder=3) #remove to have more chiseling
            
            _data_points_chisel = np.apply_along_axis(rma.TxyzRxyz_2_TxyzQwxyz, 1,_data_points_chisel)
            _data_points_chisel = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_chisel)
            _pose_waypoints_chisel = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_chisel)
            _pose_waypoints_chisel = _pose_waypoints_chisel.tolist()

            # _data_points_gripper = next_actions['data_gripper']
            _data_points_gripper = dft.filter_outliers_verbose(_data_points_gripper, threshold=1.5)
            # _data_points_gripper = dft.smooth_waypoints_sg(_data_points_gripper, window_length=10, polyorder=3)
    
            _data_points_gripper = np.apply_along_axis(rma.TxyzRxyz_2_TxyzQwxyz, 1,_data_points_gripper)
            _data_points_gripper = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_gripper)
            _pose_waypoints_gripper = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_gripper)
            _pose_waypoints_gripper = _pose_waypoints_gripper.tolist()

            # CARTESIAN_MSE_THRESHOLD = 0.0002
            CARTESIAN_MSE_THRESHOLD = 1
            #TODO - play with the threshold parameters
            if kg: kg_plan_handle = plan_client_cartesian(kg, _pose_waypoints_chisel, CARTESIAN_MSE_THRESHOLD, max_attemps=1,
                                                          max_step=0.001, jump_threshold = 0.0, revolute_jump_threshold=0.0, 
                                                          slowness_factor=SLOWNESS_FACTOR)
            
            if kb: kb_plan_handle = plan_client_cartesian(kb, _pose_waypoints_gripper, CARTESIAN_MSE_THRESHOLD, max_attemps=1, 
                                                          slowness_factor=SLOWNESS_FACTOR)


            EXECUTE_FLAG = 'y' 
            # EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()
            
            if EXECUTE_FLAG == 'y':
                if kg:kg.execute_joint_traj(kg_plan_handle['trajectory'])
                if kb: kb.execute_joint_traj(kb_plan_handle['trajectory'])

                _tick = time.time()
                execution_finished = False
                while not execution_finished:
                    mse_kg = 0
                    if kg: mse_kg = get_mse_planend_current(kg, kg_plan_handle['trajectory'])

                    mse_kb = 0
                    if kb: mse_kb = get_mse_planend_current(kb, kb_plan_handle['trajectory'])

                    if (mse_kg < 0.0002) and (mse_kb < 0.0002): execution_finished = True
                    
                    _tock = time.time()
                    if _tock - _tick > EXECUTION_TIMEOUT: print("Timeout: Execution not finished"); break

                    time.sleep(0.01)
                    
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
