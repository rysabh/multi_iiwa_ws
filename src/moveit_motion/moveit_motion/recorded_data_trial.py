from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from submodules.wait_for_message import wait_for_message
import pickle
import csv

class MoveGroupActionClientNode(Node):
    def __init__(self, node_name, move_group_to_run="arm"):
        super().__init__(node_name)

        self.action_server = "move_action"
        self.move_group_name = move_group_to_run
        self.base = f"{move_group_to_run}_link_0"
        self.end_effector = f"{move_group_to_run}_link_ee"
        # self.follow_joint_trajectory_for_move_group = f"/{move_group_to_run}/joint_trajectory_controller/follow_joint_trajectory"

        # MoveIt action client
        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.action_server
        )
        if not self.move_group_action_client.wait_for_server(timeout_sec=3):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )
        
        # Inverse kinematics client
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        if not self.ik_client.wait_for_service(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to service {self.ik_client.srv_name}."
            )
    

        
    def request_inverse_kinematics(self, pose: Pose) -> JointState:
        r"""Request inverse kinematics for a given pose."""
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.move_group_name
        # tf_prefix = self.get_namespace().removeprefix("/")

        # print(f"\n\n------\n\n{tf_prefix}\n\n_____\n\n")

        request.ik_request.pose_stamped.header.frame_id = f"{self.base}"

        print(f"\n\n------\n\n{request.ik_request.pose_stamped.header.frame_id}\n\n_____\n\n")

        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.avoid_collisions = True
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("Inverse kinematics service call failed")
            return
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Failed to compute inverse kinematics: {response.error_code.val}"
            )
            return
        return response.solution.joint_state
    
    
    def move_to_joint_pos(self, target_joint_state=None):
        if target_joint_state is None:
            return
        joint_state = target_joint_state

        # Prepare goal
        goal = MoveGroup.Goal()
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10
        goal.request.group_name = self.move_group_name
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.max_velocity_scaling_factor = 0.05

        # Set joint constraints
        goal.request.goal_constraints.append(
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=joint_state.name[i],
                        position=joint_state.position[i],
                        tolerance_above=0.001,
                        tolerance_below=0.001,
                        weight=1.0,
                    )
                    for i in range(len(joint_state.name))
                ]
            )
        )
        # Send goal
        goal_future = self.move_group_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return

        # Wait for result
        self.get_logger().info("MoveGroup goal accepted")
        self.get_logger().info("Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result


        # trajectory = result.planned_trajectory
        # print(f"goal trajectry is \n*********************\n\n\n{trajectory}\n\n\n**********************\n")
        
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveGroup action failed: {result.error_code.val}")
            return
        self.get_logger().info("MoveGroup action succeeded")

        # self.joint_trajectories.append(result.planned_trajectory.joint_trajectory)
        return result.planned_trajectory.joint_trajectory
    
    def get_current_robot_joint_state(self):
        _MSG_RECEIVED_BOOL, current_joint_state = wait_for_message(
            JointState, self, f"/{self.move_group_name}/joint_states", time_to_wait=1.0
        )
        if not _MSG_RECEIVED_BOOL:
            self.get_logger().error("Failed to get current joint state")
            return None
        return current_joint_state
    
    def modify_joint_state_for_sim_robot(self, robot_joint_state):
        _prefix_to_add = f"{self.move_group_name}"
        _name_of_all_joints = robot_joint_state.name
        for i in range(len(_name_of_all_joints)):
            _current_joint_name = _name_of_all_joints[i]
            _new_joint_name = f"{_prefix_to_add}_{_current_joint_name}"
            _name_of_all_joints[i] = _new_joint_name
        robot_joint_state.header.frame_id = "world"
        return robot_joint_state
    
    def connect_2_real_robot(self):
        # _follow_joint_trajectory_for_move_group = f"/{self.move_group_name}/joint_trajectory_controller/follow_joint_trajectory"
        _follow_joint_trajectory_for_move_group = f"/kuka_green/joint_trajectory_controller/follow_joint_trajectory"
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, _follow_joint_trajectory_for_move_group)
        if not self.trajectory_client.wait_for_server(timeout_sec=3):
            raise RuntimeError(
                f"Couldn't connect to Action Server {_follow_joint_trajectory_for_move_group}."
            )

    def execute_trajectory_on_real_robot(self, joint_trajectory: JointTrajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory
        goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            return

        # Wait for result
        self.get_logger().info("Trajectory goal accepted")
        self.get_logger().info("Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        ##TODO
        # result = result_future.result()
        # if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        #     self.get_logger().error(f"Trajectory execution failed: {result.error_code}")
        #     return
        # self.get_logger().info("Trajectory execution succeeded")

        
def main():
    rclpy.init()

    #kuka_blue
    move_group_action_client_node_sim_blue = MoveGroupActionClientNode(
        node_name="move_group_action_client_node_sim_blue",
        move_group_to_run="kuka_blue"
    )
    move_group_action_client_node_sim_blue.connect_2_real_robot()
    #kuka_green
    move_group_action_client_node_sim_green = MoveGroupActionClientNode(
        node_name="move_group_action_client_node_sim_green",
        move_group_to_run="kuka_green"
    )
    move_group_action_client_node_sim_green.connect_2_real_robot()
    #dual_kuka
    move_group_action_client_node_sim_dual = MoveGroupActionClientNode(
        node_name="move_group_action_client_node_sim_dual",
        move_group_to_run="dual_arm"
    )


    # kuka_blue_current_joint_state = move_group_action_client_node_sim_blue.get_current_robot_joint_state()
    # move_group_action_client_node_sim_blue.modify_joint_state_for_sim_robot(kuka_blue_current_joint_state)
    # move_group_action_client_node_sim_blue.move_to_joint_pos(kuka_blue_current_joint_state)

    # #kuka_green
    # kuka_green_current_joint_state = move_group_action_client_node_sim_green.get_current_robot_joint_state()
    # move_group_action_client_node_sim_green.modify_joint_state_for_sim_robot(kuka_green_current_joint_state)
    # move_group_action_client_node_sim_green.move_to_joint_pos(kuka_green_current_joint_state)


    # poses_green = [
    #     Pose(
    #         position=Point(x=1.0782, y=0.97107, z=1.113),
    #         orientation=Quaternion(x=0.00023184, y=0.00024208, z=0.00036548, w=1.0),
    #     ),
    #     Pose(
    #         position=Point(x=1.0541, y=0.85627, z=1.0708),
    #         orientation=Quaternion(x=0.00026886, y=0.00019192, z=0.00043846, w=1.0),
    #     )
    # ]

    # poses_blue = [
    #     Pose(
    #         position=Point(x=0.053813, y=-0.0006736, z=1.1839),
    #         orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    #     ),
    #     Pose(
    #         position=Point(x=0.082387, y=-0.085811, z=1.0574),
    #         orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    #     )
    # ]

    # poses_blue = [
    #     Pose(
    #         position=Point(x=0.053813, y=-0.0006736, z=1.1839),
    #         orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    #     )
    # ]

    poses_blue = []
    csv_file = '/home/cam/Downloads/scooper_pose_take/scopper_pose_take_195.csv'
    with open(csv_file, newline='') as file:
        reader = csv.DictReader(file)
        for index, row in enumerate(reader):
                # if index < 4:
                    # continue
                pos_X = float(row['x']) * 0.001
                pos_Y = float(row['y']) * 0.001
                pos_Z = float(row['z']) * 0.001
                ori_x = float(row['qx'])
                ori_y = float(row['qy'])
                ori_z = float(row['qz'])
                ori_w = float(row['w'])

                position = Point(x=pos_X, y=pos_Y, z=pos_Z)
                orientation = Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)

                pose = Pose(position=position, orientation=orientation)
                poses_blue.append(pose)
                

    poses = {
            # "kuka_green": poses_green, 
             "kuka_blue": poses_blue
             }

    green_traj_points = []
    blue_traj_points = []

    def save_trajectory(trajectory_data, filename):
        dir_path="/home/cam/multi_iiwa_ws/src/moveit_motion/moveit_motion/robot_trajectories"
        full_path = os.path.join(dir_path, filename)
        with open(full_path, 'wb') as file:
            pickle.dump(trajectory_data, file)
    
    for i in range(len(poses['kuka_blue'])):
        inverse_blue = move_group_action_client_node_sim_blue.request_inverse_kinematics(poses['kuka_blue'][i])
        # modify_blue = erase_zero_values(joint_values=inverse_blue, move_group_name='kuka_blue')

        inverse_green = move_group_action_client_node_sim_green.request_inverse_kinematics(poses['kuka_blue'][i])
        # modify_green = erase_zero_values(joint_values=inverse_green, move_group_name='kuka_green')


        dual_inverse_list = JointState()
        dual_inverse_list.effort = inverse_blue.effort
        dual_inverse_list.header = inverse_blue.header
        dual_inverse_list.name = inverse_blue.name
        dual_inverse_list.position = inverse_blue.position[:7] + inverse_green.position[7:]
        dual_inverse_list.velocity = inverse_blue.velocity
        _planned_joint_trajectory = move_group_action_client_node_sim_dual.move_to_joint_pos(dual_inverse_list)
        # _planned_joint_trajectory = move_group_action_client_node_sim_dual.move_to_joint_pos(inverse_blue)


        # green_jt = copy.deepcopy(_planned_joint_trajectory)
        # green_jt.header.frame_id = ''
        # green_jt.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']

        # for i in range(len(green_jt.points)):
        #     green_jt.points[i].positions = green_jt.points[i].positions[0:7]
        

        for point in _planned_joint_trajectory.points:
            green_waypoint = JointTrajectoryPoint()
            blue_waypoint = JointTrajectoryPoint()
            robot_dof = int(len(point.positions)/2)
            green_waypoint.positions = point.positions[robot_dof:]
            blue_waypoint.positions = point.positions[:robot_dof]
            green_waypoint.accelerations = point.accelerations[robot_dof:]
            blue_waypoint.accelerations = point.accelerations[:robot_dof]
            green_waypoint.velocities = point.velocities[robot_dof:]
            blue_waypoint.velocities = point.velocities[:robot_dof]
            green_waypoint.effort = point.effort
            blue_waypoint.effort = point.effort
            green_waypoint.time_from_start = point.time_from_start
            blue_waypoint.time_from_start = point.time_from_start
            green_traj_points.append(green_waypoint)
            blue_traj_points.append(blue_waypoint)

            
        green_joint_traj = JointTrajectory()
        green_joint_traj.header.stamp.sec  = 0
        green_joint_traj.header.stamp.nanosec = 0
        green_joint_traj.header.frame_id = ''
        green_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
        green_joint_traj.points = green_traj_points


        blue_joint_traj = JointTrajectory()
        blue_joint_traj.header.stamp.sec  = 0
        blue_joint_traj.header.stamp.nanosec = 0
        blue_joint_traj.header.frame_id = ''
        blue_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
        blue_joint_traj.points = blue_traj_points
        save_trajectory(blue_joint_traj, f"kuka_blue_{i}.dump")
        save_trajectory(green_joint_traj, f"kuka_green_{i}.dump")
        move_group_action_client_node_sim_blue.execute_trajectory_on_real_robot(green_joint_traj)




    # def save_trajectory(trajectory_data, filename):
    #     dir_path="/home/cam/multi_iiwa_ws/src/moveit_motion/moveit_motion/robot_trajectories"
    #     full_path = os.path.join(dir_path, filename)
    #     with open(full_path, 'wb') as file:
    #         pickle.dump(trajectory_data, file)

    # save_trajectory(blue_joint_traj, "kuka_blue.dump")
    # save_trajectory(green_joint_traj, "kuka_green.dump")
    
    # move_group_action_client_node_sim_blue.execute_trajectory_on_real_robot(blue_joint_traj)

    rclpy.shutdown()

if __name__ == '__main__':
    import os
    main()