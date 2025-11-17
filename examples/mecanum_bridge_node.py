#!/usr/bin/env python3
"""
ROS2 Bridge Node for ESP32 Mecanum Robot with Encoder Odometry

This node bridges the ESP32 UART communication with ROS2 topics:
- Subscribes to /cmd_vel (geometry_msgs/Twist)
- Publishes to /odom (nav_msgs/Odometry)
- Publishes TF transform (odom -> base_link)

Requirements:
    pip3 install pyserial
    sudo apt install ros-humble-nav-msgs ros-humble-tf2-ros

Usage:
    ros2 run <your_package> mecanum_bridge_node.py
    
    Or standalone:
    python3 mecanum_bridge_node.py
"""

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time

class MecanumBridgeNode(Node):
    def __init__(self):
        super().__init__('mecanum_bridge_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
            time.sleep(2)  # Wait for ESP32 to stabilize
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise
        
        # ROS2 Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # ROS2 Subscribers
        self.twist_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        
        # Timer for reading serial data
        self.create_timer(0.01, self.read_serial_callback)  # 100Hz check rate
        
        # Statistics
        self.odom_count = 0
        self.twist_count = 0
        self.last_odom_time = self.get_clock().now()
        
        self.get_logger().info('Mecanum Bridge Node started')
        self.get_logger().info(f'Publishing odometry on /odom')
        self.get_logger().info(f'Subscribing to /cmd_vel')
    
    def twist_callback(self, msg):
        """
        Callback for /cmd_vel topic
        Converts ROS2 Twist to ESP32 UART format
        """
        try:
            # Format: TWIST,linear_x,linear_y,angular_z
            cmd = f"TWIST,{msg.linear.x:.4f},{msg.linear.y:.4f},{msg.angular.z:.4f}\n"
            self.serial.write(cmd.encode())
            
            self.twist_count += 1
            if self.twist_count % 20 == 0:  # Log every 20 commands
                self.get_logger().debug(
                    f'Sent twist: x={msg.linear.x:.2f}, '
                    f'y={msg.linear.y:.2f}, z={msg.angular.z:.2f}'
                )
        except Exception as e:
            self.get_logger().error(f'Error sending twist command: {e}')
    
    def read_serial_callback(self):
        """
        Timer callback to read serial data from ESP32
        Parses odometry messages and publishes to ROS2
        """
        try:
            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                
                if line.startswith('ODOM,'):
                    self.parse_and_publish_odom(line)
                elif line:
                    # Log other messages from ESP32
                    self.get_logger().info(f'ESP32: {line}')
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')
    
    def parse_and_publish_odom(self, line):
        """
        Parse ODOM message and publish as ROS2 Odometry
        Format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
        """
        try:
            parts = line.split(',')
            if len(parts) != 11:
                self.get_logger().warn(f'Invalid ODOM message: {line}')
                return
            
            # Parse odometry data
            x = float(parts[1])
            y = float(parts[2])
            theta = float(parts[3])
            vx = float(parts[4])      # strafe velocity (robot frame)
            vy = float(parts[5])      # forward velocity (robot frame)
            omega = float(parts[6])   # angular velocity
            
            # Create timestamp
            current_time = self.get_clock().now()
            
            # Create Odometry message
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            
            # Set position
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            
            # Set orientation (convert theta to quaternion)
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(theta / 2.0)
            
            # Set velocity (in robot frame)
            # ROS convention: x=forward, y=left, z=up
            # ESP32 sends: vx=strafe(right), vy=forward
            odom.twist.twist.linear.x = vy      # forward velocity
            odom.twist.twist.linear.y = -vx     # left velocity (negate strafe)
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = omega
            
            # Set covariance (adjust based on your robot's accuracy)
            # Order: x, y, z, rot_x, rot_y, rot_z
            pose_cov = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,    # x
                       0.0, 0.1, 0.0, 0.0, 0.0, 0.0,    # y
                       0.0, 0.0, 1e9, 0.0, 0.0, 0.0,    # z (not used)
                       0.0, 0.0, 0.0, 1e9, 0.0, 0.0,    # rot_x (not used)
                       0.0, 0.0, 0.0, 0.0, 1e9, 0.0,    # rot_y (not used)
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.2]    # rot_z (yaw)
            odom.pose.covariance = pose_cov
            
            twist_cov = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 1e9, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 1e9, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 1e9, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
            odom.twist.covariance = twist_cov
            
            # Publish odometry
            self.odom_pub.publish(odom)
            
            # Publish TF transform
            if self.publish_tf:
                t = TransformStamped()
                t.header = odom.header
                t.child_frame_id = odom.child_frame_id
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation = odom.pose.pose.orientation
                self.tf_broadcaster.sendTransform(t)
            
            # Log statistics
            self.odom_count += 1
            if self.odom_count % 20 == 0:  # Log every 20 messages (1 second at 20Hz)
                dt = (current_time - self.last_odom_time).nanoseconds / 1e9
                rate = 20.0 / dt if dt > 0 else 0.0
                self.get_logger().info(
                    f'Odom: pos=({x:.2f}, {y:.2f}), '
                    f'theta={theta:.2f}, rate={rate:.1f}Hz'
                )
                self.last_odom_time = current_time
        
        except ValueError as e:
            self.get_logger().error(f'Error parsing odometry values: {e}')
        except Exception as e:
            self.get_logger().error(f'Error publishing odometry: {e}')
    
    def destroy_node(self):
        """Clean up when node is shutting down"""
        try:
            # Stop the robot
            cmd = "TWIST,0.0,0.0,0.0\n"
            self.serial.write(cmd.encode())
            time.sleep(0.1)
            self.serial.close()
            self.get_logger().info('Serial port closed')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MecanumBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
