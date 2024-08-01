import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class RelayController(Node):
    def __init__(self, port='dev/ttyUSB0', baudrate=9600):
        super().__init__('relay_controller')
        self.ser = serial.Serial(port, baudrate)
        self.relay_state = False
        self.relay_sub = self.create_subscription(Bool, '/relay_commad', self.relay_callback, 10)
        self.get_logger().info("Relay Controller Node Initialized")

    def relay_callback(self, msg):
        self.relay_state = msg.data
        self.update_relay()

    def update_relay(self):
        if self.relay_state:
            self.ser.write(b'H')
            self.get_logger().info('Relay Turned ON')

        else:
            self.ser.write(b'L')
            self.get_logger().info('Relay Turned OFF')

def main(args=None):
    rclpy.init(args=args)
    try:
        relay_controller = RelayController()
        rclpy.spin(relay_controller)
    except serial.SerialException:
        relay_controller.get_logger().error("Failed to connect to the serial port")
    except KeyboardInterrupt:
        pass
    finally:
        relay_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()