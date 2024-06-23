from typing import List

import sys

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class MoveGroupActionClientNode(Node):
    def __init__(self, node_name: str, move_group_to_run="arm") -> None:
        super().__init__(node_name)

        self.action_server = "move_action"
        self.move_group_name = move_group_to_run
        self.base = f"{move_group_to_run}_link_0"
        self.end_effector = f"{move_group_to_run}_link_ee"

        # MoveIt action client
        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.action_server
        )
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )

        # Inverse kinematics client
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        if not self.ik_client.wait_for_service(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to service {self.ik_client.srv_name}."
            )
        
        self.follow_joint_trajectory_for_move_group = f"/{move_group_to_run}/joint_trajectory_controller/follow_joint_trajectory"
        
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, self.follow_joint_trajectory_for_move_group)


        if not self.trajectory_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to Action Server {self.follow_joint_trajectory_for_move_group}."
            )

        
        # self.joint_trajectories = []

    def modify_trajectory_for_real_robot(self, traj_to_modify):
        '''TODO: change the header FRAME_ID from world to link_0'''
        for i, joint in enumerate(traj_to_modify.joint_names): traj_to_modify.joint_names[i] = f"A{i+1}"
        traj_to_modify.header.frame_id = f"{self.move_group_name}/link_0"
        return traj_to_modify
        
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

    def move_to_pose(self, target: Pose):
        r"""Move the robot to a given pose. Do so by requesting inverse kinematics and then sending a move group action."""
        joint_state = self.request_inverse_kinematics(target)
        if joint_state is None:
            return

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
        

    # def get_joint_trajectories(self):
    #     return self.joint_trajectories
    
    # def save_joint_trajectories_to_yaml(file_path: str, trajectories: List):
    #     with open(file_path, 'w') as file:
    #         yaml.dump(trajectories, file)

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
            result = result_future.result()
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(f"Trajectory execution failed: {result.error_code}")
                return
            self.get_logger().info("Trajectory execution succeeded")

def main(args: List = None) -> None:

    rclpy.init(args=args)
    
    # move_group_action_client_node_sim_green = MoveGroupActionClientNode(
    #     node_name="move_group_action_client_node_sim_green",
    #     move_group_to_run="kuka_green"
    # )

    move_group_action_client_node_sim_blue = MoveGroupActionClientNode(
        node_name="move_group_action_client_node_sim_blue",
        move_group_to_run="kuka_blue"
    )


    # List of poses to move the robot to
    poses = [
        Pose(
            position=Point(x=0.33736, y=-0.087311, z=0.6165),
            orientation=Quaternion(x=0.29244,y=0.95605,z=-0.0068769,w=0.019776),
        ),
        Pose(
            position=Point(x=0.33736, y=-0.087311, z=0.6165),
            orientation=Quaternion(x=0.44918, y=0.89319, z=-0.0040065, w=0.020923),
        ),
        # Pose(
        #     position=Point(x=0.4, y=0.3, z=0.8),
        #     orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),
        # ),
        # Add more poses as needed
    ]

    joint_trajectories_green = []
    joint_trajectories_blue = []

    # for i in range(len(poses)):
    #     if i==1:break
    #     green_traj_i = move_group_action_client_node_sim_green.move_to_pose(poses[i])
    #     move_group_action_client_node_sim_green.modify_trajectory_for_real_robot(traj_to_modify=green_traj_i)
    #     joint_trajectories_green.append(green_traj_i)
    #     move_group_action_client_node_sim_green.trajectory_client.send_goal_async(green_traj_i)

    
    for i in range(len(poses)):
        if i==2:break
        blue_traj_i = move_group_action_client_node_sim_blue.move_to_pose(poses[i])
        move_group_action_client_node_sim_blue.modify_trajectory_for_real_robot(traj_to_modify=blue_traj_i)
        joint_trajectories_blue.append(blue_traj_i)
        
        print(i)
        if i==1:
            # print(blue_traj_i)
            run_status = input("do you want to execute y/n: ")
            if str(run_status).strip().lower() == "y": move_group_action_client_node_sim_blue.execute_trajectory_on_real_robot(blue_traj_i)
            else: print("planned but not executed")
    
    
    print()



    # move_group_action_client_node_real_blue = MoveGroupActionClientNode(
    #     node_name="move_group_action_client_node_real_blue",
    #     move_group_to_run="kuka_blue"
    # )

    # move_group_action_client_node_real_green = MoveGroupActionClientNode(
    #     node_name="move_group_action_client_node_real_green",
    #     move_group_to_run="kuka_blue"
    # )

    # print("Joint Trajectories for Kuka Green:")
    # for trajectory in joint_trajectories_1:
    #     print(trajectory)

    # print("Joint Trajectories for Kuka Blue:")
    # for trajectory in joint_trajectories_2:
    #     print(trajectory)

    # trajectories = {
    #     "kuka_green": joint_trajectories_1,
    #     "kuka_blue": joint_trajectories_2
    # }

    # save_joint_trajectories_to_yaml("/home/cam/lbr-stack/src/moveit_motion/moveit_motion/joint_trajectories.yaml", trajectories)
    # print("Joint trajectories saved to joint_trajectories.yaml")

    rclpy.shutdown()


if __name__ == "__main__":
    main()