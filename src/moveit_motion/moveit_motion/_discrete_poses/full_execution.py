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
import moveit_motion.diffusion_policy_cam.submodules.data_filter as dft


def get_robot_next_actions(step_size=8):
    # Zip the data_chisel and data_gripper so we can iterate over them simultaneously
    for i in range(0, len(_data_chisel), step_size):
        data_chisel_chunk = _data_chisel[i:i+step_size]
        data_gripper_chunk = _data_gripper[i:i+step_size]
        
        # Yield the chunks for both chisel and gripper
        yield {
            'data_chisel': data_chisel_chunk,
            'data_gripper': data_gripper_chunk
        }

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

def get_mse_planend_current(_client, _trajectory):
    _current_joint_state = _client.get_current_joint_state()
    _target_joint_state = copy.deepcopy(_current_joint_state)
    _target_joint_state.position = _trajectory.joint_trajectory.points[-1].positions
    return rsmod.MSE_joint_states(_current_joint_state, _target_joint_state)
    

##------ OG full_execution.py -----------------------------------------
# KG_HOME = [-0.614, 0.634, 2.302, -1.634, 1.526, -1.549, -1.897]
# KB_HOME = [-0.590, -1.136, -2.251, 1.250, -1.929, 0.964, 0.494]

# KG_CHISEL_START = [-0.908, 1.000, 2.218, -1.330, 1.377, -1.391, -2.146]
# KB_GRIPPER_START = [-0.548, -0.289, -1.942, 1.609, -1.596, 1.258, -0.877]


#-----------------------------------------
# from full_execution_integrated.py
KG_HOME = [-1.1024725513365605, 0.8110270609209829, 2.2307813006892285, -1.4287002552588133, 1.604325087473934, -1.3662565244916118, -2.0382540102696893] #from full_execution_integrated.py
# KB_HOME = [-2.6638647431297615, -1.5463471223827439, -0.35460791496306254, -1.704449912236285, -1.6962977175622784, 1.259742901404267, 0.507759573970187] #from full_execution_integrated.py

#anti kb collision
KB_HOME = [-2.814634655497235, -1.4391204945762708, -0.578491299344867, -1.4757622621783792, -1.3718623160027494, 1.044621322393108, -0.010342925430741204]
# KG_CHISEL_START = [-1.0601302434590743, 0.8409170423409222, 2.1722489721155345, -1.3470277402052244, 1.5864082126577137, -1.331293906464206, -2.1165155976321604] #from full_execution_integrated.py
KG_CHISEL_START = [-1.1232479206995292, 0.7491314135787319, 2.3095718126808125, -1.5271257484370588, 1.6341402195290096, -1.4306376331820347, -1.906858962570551]
KB_GRIPPER_START = KB_HOME  #from full_execution_integrated.py
##-----------------------------------------

KG_END = [-1.6579662128466874, 0.8721935286964191, 2.5547452380354114, -1.814603876424333, 1.5203432120805878, -1.598732320640838, -1.8471244591419347]
KB_END = KB_HOME
##-----------------------------------------

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

    #-----------------------------#
    
    action_generator = get_robot_next_actions(step_size=STEP_SIZE)

    # if kb: move_client_ptp(kb, KB_HOME)
    # if kg: move_client_ptp(kg, KG_HOME)
    
    # if kb: move_client_ptp(kb, KB_GRIPPER_START)
    if kg: move_client_ptp(kg, KG_CHISEL_START)
    

    for next_actions in action_generator:
        ##------ Next Actions ------##
        _data_points_chisel = next_actions['data_chisel']
        _data_points_gripper = next_actions['data_gripper']
        ##-------------------------##


        _data_points_chisel = dft.filter_outliers_verbose(_data_points_chisel, threshold=2.5)
        # _data_points_chisel = dft.smooth_waypoints_sg(_data_points_chisel, window_length=4, polyorder=3) #remove to have more chiseling
        _data_points_chisel = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_chisel)
        _pose_waypoints_chisel = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_chisel)
        _pose_waypoints_chisel = _pose_waypoints_chisel.tolist()

        _data_points_gripper = dft.filter_outliers_verbose(_data_points_gripper, threshold=2.5)
        # _data_points_gripper = dft.smooth_waypoints_sg(_data_points_gripper, window_length=10, polyorder=3)
        _data_points_gripper = np.apply_along_axis(rosm.robodk_2_ros, 1, _data_points_gripper)
        _pose_waypoints_gripper = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points_gripper)
        _pose_waypoints_gripper = _pose_waypoints_gripper.tolist()

        CARTESIAN_MSE_THRESHOLD = 0.8
        
        if kg: kg_plan_handle = plan_client_cartesian(kg, _pose_waypoints_chisel, CARTESIAN_MSE_THRESHOLD, max_attemps=1,
                                                          max_step=0.001, jump_threshold = 0.0, revolute_jump_threshold=0.0, 
                                                          slowness_factor=SLOWNESS_FACTOR)
            
        if kb: kb_plan_handle = plan_client_cartesian(kb, _pose_waypoints_gripper, CARTESIAN_MSE_THRESHOLD, max_attemps=1, 
                                                          slowness_factor=SLOWNESS_FACTOR)


        EXECUTE_FLAG = input("Execute trajectory? (y/n): ").strip().lower()
        
        if EXECUTE_FLAG == input("Execute trajectory? (y/n): ").strip().lower():
            if kg: kg.execute_joint_traj(kg_plan_handle['trajectory'])
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
                if _tock - _tick > EXECUTION_TIMEOUT: print(f"Timeout {EXECUTION_TIMEOUT} seconds: Execution not finished"); break

                time.sleep(0.01)

    print("Going to home position")
    if kb: move_client_ptp(kb, KB_END)
    if kg: move_client_ptp(kg, KG_END)

    if kb: move_client_ptp(kb, KB_GRIPPER_START)
    if kg: move_client_ptp(kg, KG_CHISEL_START)

    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    STEP_SIZE = 60
    FRACTION_TO_RUN = 1.0
    SLOWNESS_FACTOR = 3
    EXECUTION_TIMEOUT = 10



    ###############################
    #---- Next Action Generator ---
    ###############################
    START_POINT = 0
    # # _file_name = "ft_012.csv"; CHISEL_START_POINT =10 #edge 3
    # _file_name = "ft_010.csv"; CHISEL_START_POINT = 10 #edge 4
    # # _file_name = "ft_025.csv"; CHISEL_START_POINT = 10 #edge 1

    # _file_name = "ft_045.csv"; CHISEL_START_POINT = 10 #edge 3
    # _file_name = "ft_023.csv"; CHISEL_START_POINT = 10 #edge 4
    # _file_name = "ft_030.csv"; CHISEL_START_POINT = 10 #edge 1

    # _file_name = "ft_046.csv"; CHISEL_START_POINT = 10 #edge 3
    # _file_name = "ft_111.csv"; CHISEL_START_POINT = 10 #edge 4
    # _file_name = "ft_125.csv"; CHISEL_START_POINT = 10 #edge 1

    # _file_name = "ft_020.csv"; CHISEL_START_POINT = 10 #edge 3
    # _file_name = "ft_190.csv"; CHISEL_START_POINT = 10 #edge 4
    # _file_name = "ft_130.csv"; CHISEL_START_POINT = 10 #edge 1
    

    # path = f'no-sync/edge_3/{_file_name}'
    
    ######### DELETE ######################
    path = "/home/cam/Documents/raj/diffusion_policy_cam/no-sync/turn_table_chisel/trimmed_traj/transformed/extra_transform/csvs_1000/ft_020_edge_3_step_2_rotated_new_neg_28_498750798559428.csv"
    CHISEL_START_POINT = 10
    ##-------------------------------------##

    data_loader = cfp.DataParser.from_euler_file(file_path = path, target_fps= 60, filter=False, window_size=30, polyorder=3)
    #while not input is to quit
    

    _data_chisel = data_loader.get_rigid_TxyzQwxyz()['chisel'][CHISEL_START_POINT:]
    _data_gripper = data_loader.get_rigid_TxyzQwxyz()['gripper']

    # define int(FRACTION_TO_RUN*len(_data_chisel)
    index_ = int(FRACTION_TO_RUN*len(_data_chisel))

    _data_chisel = _data_chisel[:index_]
    _data_gripper = _data_gripper[:index_]
    _data_times = data_loader.get_time()[:index_]

    print(f"Number of data points for chisel: {len(_data_chisel)}")
    print(f"Number of data points for gripper: {len(_data_gripper)}")
    print(f"Number of data points for time: {len(_data_times)}")
    #-----------------------------#
    

    main()
