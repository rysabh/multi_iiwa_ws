import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory


class FollowJointActionClient(Node):
    def __init__(self, node_name: str, move_group_to_run="arm") -> None:
        super().__init__(node_name)

        self.follow_joint_trajectory_for_move_group = f"/{move_group_to_run}/joint_trajectory_controller/follow_joint_trajectory"
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, self.follow_joint_trajectory_for_move_group)
        if not self.trajectory_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to Action Server {self.follow_joint_trajectory_for_move_group}."
            )


    def execute_trajectory_on_real_robot(self, joint_trajectory):
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


def main(move_group_2_run) -> None:
    rclpy.init()

    dir_path="/home/battery/multi_iiwa_ws/src/moveit_motion/moveit_motion/robot_trajectories"
    full_path = os.path.join(dir_path, f"{move_group_2_run}.dump")
    with open(full_path, 'rb') as file:
        robot_joint_traj = pickle.load(file)

    
    follow_joint_traj_node = FollowJointActionClient(
        node_name=f"follow_joint_traj_node_{move_group_2_run}",
        move_group_to_run=move_group_2_run
    )
    

    run_status = input(f"{follow_joint_traj_node.move_group_name}: do you want to execute y/n: ")
    if str(run_status).strip().lower() == "y":
        follow_joint_traj_node.execute_trajectory_on_real_robot(robot_joint_traj)
    else: 
        print("not executed")
    

    rclpy.shutdown()




if __name__ == "__main__":
    import pickle
    import os
    main()
