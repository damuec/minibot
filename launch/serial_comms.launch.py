#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.get_logger().info("Connected to ESP32")
        self.wheelbase = 0.15
        self.max_steer_angle = 0.5
        self.max_throttle = 200
        self.max_linear_vel = 0.5

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        if abs(linear_vel) > 0.01 and abs(angular_vel) > 0.01:
            radius = linear_vel / angular_vel
            steering_angle = math.atan(self.wheelbase / radius)
        else:
            steering_angle = 0.0
        steering_angle = max(min(steering_angle, self.max_steer_angle), -self.max_steer_angle)
        throttle = int((linear_vel / self.max_linear_vel) * self.max_throttle)
        throttle = max(min(throttle, self.max_throttle), -self.max_throttle)
        command_str = f"T{throttle} S{steering_angle:.2f}\n"
        self.serial_port.write(command_str.encode('utf-8'))
        self.get_logger().debug(f"Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f} -> Throttle: {throttle}, Steer: {steering_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    ackermann_driver = AckermannDriver()
    rclpy.spin(ackermann_driver)
    ackermann_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    