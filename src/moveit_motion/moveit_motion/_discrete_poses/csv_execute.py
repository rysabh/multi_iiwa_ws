#NOT WORKING
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from moveit_motion.ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from moveit_motion.ros_submodules.MoveitInterface import MoveitInterface

import moveit_motion.ros_submodules.ros_math as rosm
import numpy as np
import moveit_motion.diffusion_policy_cam.submodules.cleaned_file_parser as cfp
import moveit_motion.ros_submodules.ros_math as rm
import csv
from math import pi


def main(_robot_name, _file_name):

    rclpy.init()
    
    # client = MoveitInterface(node_name=f"client_{_robot_name}",     
    #                               move_group_name=_robot_name, # arm # kuka_g/b..   #-> required for motion planning
    #                               remapping_name="",           # lbr # ""          #-> required for service and action remapping
    #                               prefix=_robot_name,          # ""  # kuka_g/b..   #-> required for filtering joint states and links
    #                              )
    
    client = MoveitInterface(node_name=f"client_{_robot_name}",     
                                move_group_name=_robot_name, # arm # kuka_g/b..   #-> required for motion planning
                                remapping_name=_robot_name,           # lbr # ""          #-> required for service and action remapping
                                prefix="",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                )
    

    cjs = client.get_current_joint_state()
    
    rosm.joint_state_2_list(cjs, verbose=True)
    
    '''
    # path = 'no-sync/ik_results_1_degrees.csv'
    # path = 'no-sync/2024-08-28_15-33-26.csv'
    # with open(path, 'r') as f:
    #     reader = csv.reader(f)
    #     _header = next(reader)
    #     data = []
    #     # convert degrees to radians
    #     for row in reader:
    #         data.append([float(i)*pi/180 for i in row])
    
    path = f'no-sync/edge_3/{_file_name}'
    data = cfp.DataParser.from_quat_file(file_path = path, target_fps= 30, filter=False, window_size=5, polyorder=3)
    
    item_of_req = 'chisel' if _robot_name == 'kuka_green' else 'gripper'

    _data = data.get_rigid_TxyzQwxyz()[item_of_req][20:]

    FRACTION_TO_RUN = 1.0
    
    _len_data = len(_data)
    _data_points = _data[:int(FRACTION_TO_RUN*_len_data)]
    print(f"Number of data points: {len(_data_points)}")

    _data_points = np.apply_along_axis(rm.robodk_2_ros, 1, _data_points)
    _pose_waypoints = np.apply_along_axis(rosm.TxyzQxyzw_2_Pose, 1, _data_points)
    _pose_waypoints = _pose_waypoints.tolist()
    _pose_time_stamps = data.get_time()
    #convert to float
    _pose_time_stamps = [float(i) for i in _pose_time_stamps]

    # SEGMENT_LENGTH = 5

    # for i in range(0, len(_pose_waypoints), SEGMENT_LENGTH):
    #     _cartesian_plan = client.get_cartesian_spline_plan(waypoints= _pose_waypoints[i:i+SEGMENT_LENGTH],
    #                                                        planning_frame='world')
    #     client.execute_joint_traj(_cartesian_plan)
        
    
    
    # fcp = rosm.
    # fjs = client.get_best_ik(target_pose=_pose_waypoints[0], current_joint_state=cjs, attempts=300)

    # initial_plan = client.get_joint_ptp_plan(start_joint_state= cjs, target_joint_state=fjs, planner_type= "ompl", attempts= 100)
    
    # client.execute_joint_traj(initial_plan)

    # joint_robot_traj = rosm.joint_points_2_trajectory(points = data,
    #                                             times= None,
    #                                             header_frame_id= "world",
    #                                             joint_names= cjs.name,
    #                                             sampling_rate= 5.0)

    # client.execute_joint_traj(joint_robot_traj)

    _cartesian_plan_handle= client.get_cartesian_spline_plan(waypoints= _pose_waypoints, time_stamps=[], planning_frame='world',
                                                             _planner_type= "cartesian_interpolator",
                                                             max_step = 0.01, jump_threshold = 0.0, avoid_collisions = False, 
                                                             attempts=1
                                                             )

    # _cartesian_plan_handle= client.get_cartesian_spline_plan(waypoints= _pose_waypoints, time_stamps=[], planning_frame='world',
    #                                                          _planner_type= "cartesian_sequence",
    #                                                          blend_radius = 0.01, max_acceleration_scaling_factor=0.1, max_velocity_scaling_factor=0.1,
    #                                                          num_planning_attempts=1, attempts=1
    #                                                          )
    
    # print(client.sequence_srv_name_)
    # _cartesian_plan_handle = client.get
    _cartesian_plan = _cartesian_plan_handle['trajectory']
    _fraction = _cartesian_plan_handle['fraction']
    print(f"Fraction of path executed: {_fraction}")
    stop_flag = _cartesian_plan_handle['stop_flag']
    print(f"Stop flag: {stop_flag}")

    # ADD_TIMES_FLAG = len(time_stamps) == len(waypoints)
    # if len(time_stamps) > 0 and not ADD_TIMES_FLAG:
    #     self.get_logger().error("Invalid time_stamps provided")
    #     return None
    # if ADD_TIMES_FLAG and _response_handle['stop_flag']:
    #     _completed_time_steps = int(len(time_stamps) * _fraction) 
    #     _trajectory = rosm.interpolate_trajectory_timestamps(_trajectory, time_stamps[:_completed_time_steps], scaling_factor=0.5)

    ############################################################
    #----------- Trajectory Execution --------------------------
    ############################################################
    execute_flag = input("Execute trajectory? (y/n): ")
    if execute_flag == 'y':
        client.execute_joint_traj(_cartesian_plan)
    else:
        print("Trajectory not executed")
    ############################################################

    print(f"Fraction of path executed: {_fraction}")
    '''
    rclpy.shutdown()


if __name__ == '__main__':
    import sys

    _robot_name = sys.argv[1] if len(sys.argv) > 1 else "kuka_blue"
    _file_name = sys.argv[2] if len(sys.argv) > 2 else "ft_010.csv"

    main(_robot_name, _file_name)


