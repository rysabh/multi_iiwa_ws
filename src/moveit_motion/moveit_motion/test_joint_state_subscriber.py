from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriberNode(Node):
    def __init__(self, node_name: str = "joint_state_subscriber_node", move_group_to_run="kuka_blue") -> None:
        super().__init__(node_name)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            f"/{move_group_to_run}/joint_states",
            self.joint_state_callback,
            10
        )
        self.joint_state_received = False

    def joint_state_callback(self, msg: JointState):
        print("--running the callback to move to robot's initial position")
        if not self.joint_state_received:
            self.get_logger().info(f" Received joint states: {msg}")
            self.joint_state_received = True
            self.destroy_subscription(self.joint_state_subscriber)
            # rclpy.shutdown()


def main(args: List = None) -> None:
    rclpy.init(args=args)
    joint_state_subscriber_node = JointStateSubscriberNode()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(joint_state_subscriber_node)

    try:
        executor.spin_once(timeout_sec=1.0)
    finally:
        # rclpy.shutdown()
        executor.remove_node(joint_state_subscriber_node)
        joint_state_subscriber_node.destroy_node()


if __name__ == "__main__":
    main()
