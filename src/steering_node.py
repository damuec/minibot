#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('cmd_vel_topic', 'nav_vel')  # Changed to nav_vel
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        self.command_timeout = self.get_parameter('command_timeout').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.connect_serial()
        
        # Create subscription to nav_vel
        self.subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info(f'Subscribed to {cmd_vel_topic} topic')
        
        # Initialize last command time
        self.last_command_time = self.get_clock().now().nanoseconds / 1e9
        
        # Create timer for safety check
        self.timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Steering node initialized')

    # ... rest of the code remains the same ...