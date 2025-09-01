#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class AckermannDriver(Node):
    def __init__(self):
        try:
            super().__init__('ackermann_driver')
            
            # Parameters
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('serial_port', '/dev/esp32'),
                    ('baudrate', 115200),
                    ('wheelbase', 0.325),
                    ('max_steer_angle', 0.5236),  # 30° in radians
                    ('max_throttle', 200),
                    ('max_linear_vel', 1.0),
                ]
            )
            
            # Retrieve parameters
            self.serial_port = self.get_parameter('serial_port').value
            self.baudrate = self.get_parameter('baudrate').value
            self.wheelbase = self.get_parameter('wheelbase').value
            self.max_steer_angle = self.get_parameter('max_steer_angle').value
            self.max_throttle = self.get_parameter('max_throttle').value
            self.max_linear_vel = self.get_parameter('max_linear_vel').value
            
            # Initialize serial connection
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.get_logger().info(f"Connected to ESP32 on {self.serial_port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect to ESP32: {e}")
                self.ser = None
            
            # Create subscription to cmd_vel
            self.subscription = self.create_subscription(
                Twist,
                'cmd_vel',
                self.cmd_vel_callback,
                10)
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize node: {e}")
            raise

    def cmd_vel_callback(self, msg):
        # Your existing cmd_vel processing code here
        pass

    def __del__(self):
        if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    try:
        ackermann_driver = AckermannDriver()
        rclpy.spin(ackermann_driver)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ackermann_driver' in locals():
            ackermann_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()