#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB1'),
                ('baudrate', 115200),
                ('wheelbase', 0.325),
                ('max_steer_angle', 0.4189),
                ('max_throttle', 200),
                ('max_linear_vel', 0.5),
            ]
        )
        
        # Retrieve parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        
        # Initialize serial connection
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
        self.get_logger().info(f"Successfully connected to ESP32 on {self.serial_port}")
        
        # Create Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscription to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Timer for reading serial and publishing odometry
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
    def cmd_vel_callback(self, msg):
        # Calculate steering angle and throttle from Twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Calculate steering angle (convert angular velocity to steering angle)
        if abs(linear_vel) > 0.001:
            steering_angle = math.atan2(angular_vel * self.wheelbase, linear_vel)
            steering_angle = max(min(steering_angle, self.max_steer_angle), -self.max_steer_angle)
        else:
            steering_angle = 0.0
        
        # Calculate throttle (convert linear velocity to throttle value)
        throttle = int((linear_vel / self.max_linear_vel) * self.max_throttle)
        throttle = max(min(throttle, self.max_throttle), -self.max_throttle)
        
        # Send command to ESP32
        command = f"{steering_angle},{throttle}\n"
        self.ser.write(command.encode())
        
    def timer_callback(self):
        # Read data from ESP32 (assuming it sends encoder ticks or velocity data)
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            try:
                # Parse the data from ESP32 (adjust this based on what your ESP32 sends)
                # Example format: "linear_velocity,angular_velocity"
                data = line.split(',')
                if len(data) == 2:
                    linear_vel = float(data[0])
                    angular_vel = float(data[1])
                    
                    # Update odometry
                    current_time = self.get_clock().now()
                    dt = (current_time - self.last_time).nanoseconds / 1e9
                    self.last_time = current_time
                    
                    # Calculate position and orientation using velocity data
                    delta_x = linear_vel * math.cos(self.th) * dt
                    delta_y = linear_vel * math.sin(self.th) * dt
                    delta_th = angular_vel * dt
                    
                    self.x += delta_x
                    self.y += delta_y
                    self.th += delta_th
                    
                    # Publish odometry message
                    odom = Odometry()
                    odom.header.stamp = current_time.to_msg()
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'base_link'
                    
                    # Set position
                    odom.pose.pose.position.x = self.x
                    odom.pose.pose.position.y = self.y
                    odom.pose.pose.position.z = 0.0
                    
                    # Set orientation (convert yaw to quaternion)
                    from geometry_msgs.msg import Quaternion
                    import tf_transformations
                    q = tf_transformations.quaternion_from_euler(0, 0, self.th)
                    odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                    
                    # Set velocity
                    odom.twist.twist.linear.x = linear_vel
                    odom.twist.twist.angular.z = angular_vel
                    
                    self.odom_pub.publish(odom)
                    
                    # Publish transform
                    t = TransformStamped()
                    t.header.stamp = current_time.to_msg()
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.translation.z = 0.0
                    t.transform.rotation = odom.pose.pose.orientation
                    
                    self.tf_broadcaster.sendTransform(t)
                    
            except ValueError:
                self.get_logger().warn(f"Received invalid data: {line}")
                
def main(args=None):
    rclpy.init(args=args)
    ackermann_driver = AckermannDriver()
    rclpy.spin(ackermann_driver)
    ackermann_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()