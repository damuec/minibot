
        #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class AckermannDriver(Node):

    def __init__(self):
        super().__init__('ackermann_driver') # Node name
        
        # Declare parameters for easy tuning without code changes
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheelbase', 0.15) # meters
        self.declare_parameter('max_steer_angle', 0.5) # radians
        self.declare_parameter('max_throttle', 200) # PWM value
        self.declare_parameter('max_linear_vel', 0.5) # m/s

        # Retrieve parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steer_angle = self.get_parameter('max_steer_angle').get_parameter_value().double_value
        self.max_throttle = self.get_parameter('max_throttle').get_parameter_value().integer_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value

        # Setup serial connection with error handling
        try:
            self.serial_connection = serial.Serial(port=serial_port, baudrate=baudrate, timeout=0.1)
            self.get_logger().info(f"Successfully connected to ESP32 on {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port {serial_port}: {e}")
            rclpy.shutdown()
            return

        # Create subscription to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel', 
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Ackermann driver node started and listening on cmd_vel")

    def listener_callback(self, msg):
        """
        Callback function for cmd_vel messages.
        Converts Twist to throttle and steering angle, then sends to ESP32.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate steering angle (Ackermann kinematics)
        if abs(linear_x) > 0.01 and abs(angular_z) > 0.01:
            radius = linear_x / angular_z
            steering_angle = math.atan(self.wheelbase / radius)
        else:
            steering_angle = 0.0
            
        # Constrain steering angle to physical limits
        steering_angle = max(min(steering_angle, self.max_steer_angle), -self.max_steer_angle)
        
        # Calculate throttle (simple linear mapping)
        throttle = int((linear_x / self.max_linear_vel) * self.max_throttle)
        throttle = max(min(throttle, self.max_throttle), -self.max_throttle)
        
        # Format and send command
        command = f"T{throttle} S{steering_angle:.3f}\n"
        try:
            self.serial_connection.write(command.encode('utf-8'))
            #self.get_logger().info(f"Sent: {command.strip()}") # Uncomment for debug
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    ackermann_driver = AckermannDriver()
    rclpy.spin(ackermann_driver)
    ackermann_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()