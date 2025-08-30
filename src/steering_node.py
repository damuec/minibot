#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
from geometry_msgs.msg import Quaternion
import tf_transformations

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/esp32'),
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
        self.last_cmd_time = self.get_clock().now()
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Successfully connected to ESP32 on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32: {e}")
            self.ser = None
        
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
        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
    def cmd_vel_callback(self, msg):
        # Store the current velocities for odometry calculation
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        
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
        if self.ser is not None and self.ser.is_open:
            command = f"T{throttle} S{steering_angle}\n"
            self.get_logger().debug(f"Sent command to ESP32: {command.strip()}")
            try:
                self.ser.write(command.encode())
                # Wait for acknowledgment (optional)
                # response = self.ser.readline().decode('utf-8').strip()
                # if response == "ACK":
                #     self.get_logger().debug("ESP32 acknowledged command")
            except Exception as e:
                self.get_logger().error(f"Failed to send command to ESP32: {e}")
        
    # Properly indented timer_callback method
    def timer_callback(self):
        current_time = self.get_clock().now()
        
        # Read data from ESP32
        if self.ser is not None and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').rstrip()
                self.get_logger().info(f"Received from ESP32: {line}")  # Debug log
                
                data = line.split(',')
                if len(data) == 2:
                    linear_vel = float(data[0])
                    angular_vel = float(data[1])
                    
                    # Update odometry
                    dt = (current_time - self.last_time).nanoseconds / 1e9
                    self.last_time = current_time
                    
                    # Calculate position and orientation using velocity data
                    delta_x = linear_vel * math.cos(self.th) * dt
                    delta_y = linear_vel * math.sin(self.th) * dt
                    delta_th = angular_vel * dt
                    
                    self.x += delta_x
                    self.y += delta_y
                    self.th += delta_th
                    
                    # Normalize the angle
                    self.th = math.atan2(math.sin(self.th), math.cos(self.th))
                    
                    # Publish odometry message
                    odom = Odometry()
                    odom.header.stamp = current_time.to_msg()
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'base_footprint'
                    
                    # Set position
                    odom.pose.pose.position.x = self.x
                    odom.pose.pose.position.y = self.y
                    odom.pose.pose.position.z = 0.0
                    
                    # Set orientation (convert yaw to quaternion)
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
                    t.child_frame_id = 'base_footprint'
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.translation.z = 0.0
                    t.transform.rotation = odom.pose.pose.orientation
                    
                    self.tf_broadcaster.sendTransform(t)
                    self.get_logger().info(f"Published odometry: x={self.x}, y={self.y}, th={self.th}")
                    
            except (ValueError, UnicodeDecodeError) as e:
                self.get_logger().warn(f"Received invalid data: {line}, error: {e}")
        else:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            q = tf_transformations.quaternion_from_euler(0, 0, self.th)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(t)
        
    def __del__(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
                
def main(args=None):
    rclpy.init(args=args)
    ackermann_driver = AckermannDriver()
    try:
        rclpy.spin(ackermann_driver)
    except KeyboardInterrupt:
        pass
    finally:
        ackermann_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()