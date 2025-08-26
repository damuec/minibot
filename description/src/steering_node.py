import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class AckermannDriver(Node):

    def __init__(self):
        super().__init__('ackermann_driver')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheelbase', 0.15)  # Distance between front and rear axles in meters
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value

        self.declare_parameter('max_steering_angle', 0.5)  # Max steering angle in degrees
        self.declare_parameter('max_throttle', 200)
        self.declare_parameter('max_linear_vel', 0.65) #m/s    
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Connected to serial port: {serial_port} at {baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {e}')
            rclpy.shutdown()
            return
        
        # Subscriber to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel', 
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x  # Forward velocity in m/s
        angular_velocity = msg.angular.z  # Angular velocity in rad/s
        
        if angular_velocity == 0:
            steering_angle = 0.0
        else:
            turning_radius = linear_velocity / angular_velocity
            steering_angle = math.atan(self.wheelbase / turning_radius)
        
        steering_angle_deg = math.degrees(steering_angle)
        
        command_str = f'STEER:{steering_angle_deg:.2f};SPEED:{linear_velocity:.2f}\n'
        
        try:
            self.ser.write(command_str.encode('utf-8'))
            self.get_logger().info(f'Sent command: {command_str.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error writing to serial port: {e}')