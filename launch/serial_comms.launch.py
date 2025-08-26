<<<<<<< HEAD
=======
```python
#!/usr/bin/env python3
>>>>>>> 2d0c168d62107694419eda70d928a61d3650b48f
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class AckermannDriver(Node):

    def __init__(self):
<<<<<<< HEAD
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
=======
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
>>>>>>> 2d0c168d62107694419eda70d928a61d3650b48f
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel', 
            self.listener_callback,
            10)
<<<<<<< HEAD
        self.subscription  # prevent unused variable warning
=======
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Ackermann driver node started and listening on cmd_vel")
>>>>>>> 2d0c168d62107694419eda70d928a61d3650b48f

<<<<<<< HEAD
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x  # Forward velocity in m/s
        angular_velocity = msg.angular.z  # Angular velocity in rad/s
        
        if angular_velocity == 0:
=======
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
>>>>>>> 2d0c168d62107694419eda70d928a61d3650b48f
            steering_angle = 0.0
<<<<<<< HEAD
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
=======
            
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
```
>>>>>>> 2d0c168d62107694419eda70d928a61d3650b48f