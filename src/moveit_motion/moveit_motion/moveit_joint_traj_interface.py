import sys
import csv
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

def read_joint_states_from_csv(file_path):
    joint_states = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            joint_states.append([math.radians(float(row['A1'])), math.radians(float(row['A2'])), math.radians(float(row['A3'])),
                                 math.radians(float(row['A4'])), math.radians(float(row['A5'])), math.radians(float(row['A6'])), math.radians(float(row['A7']))])
    return joint_states

def read_joint_states(file_path):
    joint_states = []
    joint_waypoints = [[3.0, 4.0, 3.0, -2.0, -2.0, 3.0, -1.0],
        [11.0, 10.0, 7.0, -5.0, -7.0, 9.0, -5.0],
        [16.0, 14.0, 19.0, -12.0, -17.0, 14.0, -16.0],
        [21.0, 20.0, 33.0, -19.0, -28.0, 20.0, -31.0], # add more waypoints here with the same format but bigger offset
        [26.0, 26.0, 47.0, -26.0, -39.0, 26.0, -46.0],
        [31.0, 32.0, 61.0, -33.0, -50.0, 32.0, -61.0],]
    for i in joint_waypoints:
            joint_states.append([math.radians(float(i[0])), math.radians(float(i[1])), math.radians(float(i[2])),
                                 math.radians(float(i[3])), math.radians(float(i[4])), math.radians(float(i[5])), math.radians(float(i[6]))])
    return joint_states

class KukaMotionPlanning(Node):
    def __init__(self):
        super().__init__('kuka_motion_planning')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/kuka_green_controller/follow_joint_trajectory')
        
        self.joint_names = ["kuka_green_A1", "kuka_green_A2", "kuka_green_A3", "kuka_green_A4", "kuka_green_A5", "kuka_green_A6", "kuka_green_A7"]
        self.joint_trajectories = read_joint_states('/home/battery/Downloads/ik_results_1_degrees.csv')
        
        self.send_goal()

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        for i, joint_values in enumerate(self.joint_trajectories):
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.time_from_start.sec = i+1 # Each point takes one second
            trajectory_msg.points.append(point)
        
        goal_msg.trajectory = trajectory_msg
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def main(args=None):
    try:
        rclpy.init(args=args)
    except Exception as e:
        print(f"Failed to initialize rclpy: {e}")
        return

    try:
        node = KukaMotionPlanning()
        rclpy.spin(node)
    except Exception as e:
        print(f"Exception during ROS2 node operation: {e}")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError as e:
            print(f"RuntimeError during shutdown: {e}")

if __name__ == '__main__':
    main()
