#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class RelayController(Node):
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        super().__init__('relay_controller')
        try:
            self.ser = serial.Serial(port, baudrate)
            self.relay_state = None  # Initialize with None to handle the first message
            self.relay_sub = self.create_subscription(Bool, '/relay_command', self.relay_callback, 10)
            self.get_logger().info("Relay Controller Node Initialized")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to the serial port: {e}")
            self.destroy_node()

    def relay_callback(self, msg):
        if msg.data != self.relay_state:
            self.relay_state = msg.data
            self.update_relay()

    def update_relay(self):
        if self.relay_state is True:
            self.ser.write(b'H')  # Send 'H' to turn relay on
            self.get_logger().info("Relay turned ON")
        elif self.relay_state is False:
            self.ser.write(b'L')  # Send 'L' to turn relay off
            self.get_logger().info("Relay turned OFF")

def main(args=None):
    rclpy.init(args=args)
    relay_controller = None
    try:
        relay_controller = RelayController()
        rclpy.spin(relay_controller)
    except serial.SerialException:
        if relay_controller:
            relay_controller.get_logger().error("Failed to connect to the serial port")
    except KeyboardInterrupt:
        pass
    finally:
        if relay_controller:
            relay_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
