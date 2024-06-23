import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
# from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_broadcaster')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = [
            'kuka_blue_A1', 'kuka_green_A6', 'kuka_green_A5', 'kuka_blue_A2',
            'kuka_green_A4', 'kuka_blue_A7', 'kuka_green_A3', 'kuka_green_A1',
            'kuka_blue_A5', 'kuka_green_A2', 'kuka_blue_A6', 'kuka_green_A7',
            'kuka_blue_A4', 'kuka_blue_A3'
        ]
        # Set position to 10 degrees (converted to radians)
        msg.position = [math.radians(10.0)] * len(msg.name)
        # Set velocity to 0 for all joints
        msg.velocity = [0.0] * len(msg.name)
        # Effort not used, set to NaN for all joints
        msg.effort = [float('nan')] * len(msg.name)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass

    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
