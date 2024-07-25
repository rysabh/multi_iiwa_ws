import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from ros_submodules.RS_submodules import save_trajectory, save_trajectory_to_csv, MSE_joint_states

from ros_submodules.MoveitInterface import MoveitInterface
from control_msgs.action import FollowJointTrajectory
from ros_submodules.wait_for_message import wait_for_message

class FollowJointAction(Node):
    def __init__(self, node: str, move_group_name:str = "arm"):
        super().__init__(node)

        self.follow_joint_traj = f"/{move_group_name}/joint_trajectory_controller/follow_joint_trajectory"
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, self.follow_joint_traj)
        if not self.trajectory_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(f"Couldn't connect to Action Server {self.follow_joint_traj}.")

    def execute_traj_on_robot(self, joint_traj):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
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

    def get_current_robot_joint_state(self):
        _MSG_RECEIVED_BOOL, current_joint_state = wait_for_message(
            JointState, self, f"/{self.move_group_name}/joint_states", time_to_wait=3.0
        )
        if not _MSG_RECEIVED_BOOL:
            self.get_logger().error("Failed to get current joint state")
            return None
        return current_joint_state

def main():
    rclpy.init()
    client = MoveitInterface(node_name="client",     
                                  move_group_name="kuka_green", # arm # kuka_g/b..   #-> required for motion planning
                                  remapping_name="",           # lbr # ""          #-> required for service and action remapping
                                  prefix="kuka_green",          # ""  # kuka_g/b..   #-> required for filtering joint states and links
                                 )
    follow_traj_client = FollowJointAction(node="follow_traj",
                                           move_group_name="kuka_green")

    
    # client = MoveitInterface(node_name="client",     
    #                               move_group_name="arm", 
    #                               remapping_name="lbr", 
    #                               prefix="")
    poses = [
        Pose(
                position=Point(x=0.6, y=0.0, z=0.6),
                orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
            ),
        Pose(
                position=Point(x=0.5, y=0.1, z=0.4),
                orientation=Quaternion(x=0.0, y=-1.0, z=0.0, w=0.0),
            ),
        Pose(
                position=Point(x=0.0, y=0.0, z=1.266),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
    ]

    robot_state = follow_traj_client.get_current_robot_joint_state()
    cjs = client.get_current_robot_pose()
    plan = client.get_joint_traj(target_joint_state=robot_state, 
                                                  start_joint_state=cjs,
                                                  planner_type="ompl")
    client.execute_joint_traj(fixed_trajectory)

    total_trajectory = []
    
    cjs = client.get_current_joint_state()
    print(cjs)
    cjs_pose = client.get_current_robot_pose()
    print(cjs_pose)
    for pose in poses:
        tjs = client.get_best_ik(target_pose=pose, current_joint_state=cjs, attempts=300)
        
        print(tjs)
        plan = client.get_joint_traj(target_joint_state=tjs, 
                                                  start_joint_state=cjs,
                                                  planner_type="ompl")
        total_trajectory.append(plan)
        cjs = tjs
    
    # combined_trajectory = client_dual.combine_trajectories(dual_spline_trajectory)
    fixed_trajectory = client.combine_trajectories(total_trajectory)
    client.execute_joint_traj(fixed_trajectory)
    execution = input("Do you want to execute? y/n")
    if execution.lower() == "y":
        follow_traj_client.execute_traj_on_robot(fixed_trajectory)
    # client_dual.execute_joint_traj(combined_trajectory)
    

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
