import rclpy
from rclpy.node import Node
import csv
import math
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from typing import Union
import moveit_motion.ros_submodules.ros_math as rosm
from moveit_motion.ros_submodules.wait_for_message import wait_for_message

def read_joint_states_from_csv(file_path):
    joint_states = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            joint_states.append([(float(row['J1'])), (float(row['J2'])), (float(row['J3'])),
                                 (float(row['J4'])), (float(row['J5'])), (float(row['J6'])), (float(row['J7']))])
    return joint_states

class GetFK(Node):
    def __init__(self, node_name, move_group_name="arm", remapping_name="lbr", prefix=""):
        super().__init__(node_name)
        self.node_name_ = node_name
        self.timeout_sec_ = 3.0
        self.move_group_name_ = move_group_name
        self.remapping_name_ = remapping_name
        self.prefix_ = prefix
        
        self.joint_states_topic_ = f"{remapping_name}/joint_states" if remapping_name else "joint_states"
        self.fk_srv_name_ = f"{remapping_name}/compute_fk" if remapping_name else "compute_fk"
        self.base_ = f"{prefix}_link_0" if prefix else f"{remapping_name}/link_0"
        self.end_effector_ = f"{prefix}_link_tcp" if prefix else 'link_tcp'
        self.joint_state_topic_ = f"{remapping_name}/joint_states" if remapping_name else "joint_states"

        self.fk_client_ = self.create_client(GetPositionFK, self.fk_srv_name_)
        if not self.fk_client_.wait_for_service(timeout_sec=self.timeout_sec_):
            self.get_logger().error(f"*** Basic Error: FK service not available -> {self.fk_client_.srv_name}.")
            exit(1)

    def get_fk(self, joint_state: JointState) -> Union[Pose, None]:
        '''
        Pose for the real robot
        '''
        # _robot_state = rosm.joint_2_robot_state(joint_state)
        # current_joint_state_set, current_joint_state = wait_for_message(
        #     JointState, self, self.joint_state_topic_, time_to_wait=1.0
        # )
        # if not current_joint_state_set:
        #     self.get_logger().error("Failed to get current joint state")
        #     return None

        current_robot_state = RobotState()
        current_robot_state.joint_state = current_joint_state
        robot_state = RobotState()
        robot_state.joint_state = joint_state
        _request = GetPositionFK.Request()
        # _request.header.frame_id = self.base_
        _request.header.frame_id = 'world'
        # _request.header.frame_id = 'lbr/link_0' #kuka_blue_link_0
        _request.header.stamp = self.get_clock().now().to_msg()
        # _request.fk_link_names.append('link_ee')
        _request.fk_link_names.append(self.end_effector_)
        _request.robot_state = _robot_state

        future = self.fk_client_.call_async(_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Failed to get FK solution")
            return None
        
        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Failed to get FK solution: {response.error_code.val}"
            )
            return None
        
        return response.pose_stamped[0].pose
    
def main():
    rclpy.init()
    kuka_blue = GetFK(node_name="client_real_kuka_blue",
                      move_group_name="kuka_blue",
                      remapping_name="kuka_blue", prefix="",)
    
    joint_states = read_joint_states_from_csv("no-sync/replay_traj_data/traj_195/waypoints.csv")
    for joint_state in joint_states:
        joint_state_msg = JointState
        print(joint_state_msg.name)
        joint_state_msg.name = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
        joint_state_msg.position = joint_state
        pose = kuka_blue.get_fk(joint_state_msg)
        if pose:
            print(pose)
    rclpy.shutdown()

if __name__ == '__main__':
    main()