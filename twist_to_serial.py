#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import sys

class TwistToSerial(Node):
    def __init__(self):
        super().__init__('twist_to_serial')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # ESP32 USB port
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('twist_topic', '/cmd_vel')
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Give ESP32 time to reset
            self.get_logger().info(f'Connected to ESP32 on {serial_port} at {baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            sys.exit(1)
        
        # Subscribe to twist messages
        self.subscription = self.create_subscription(
            Twist,
            twist_topic,
            self.twist_callback,
            10
        )
        
        self.get_logger().info(f'Listening to {twist_topic} and forwarding to ESP32')
        
        # Safety timer - send zero commands if no messages received
        self.last_message_time = time.time()
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10Hz safety check
        
    def twist_callback(self, msg):
        """Convert ROS2 Twist message to serial string and send to ESP32"""
        
        # Extract twist values
        linear_x = msg.linear.x    # forward/backward
        linear_y = msg.linear.y    # strafe left/right (usually 0 for diff drive)
        angular_z = msg.angular.z  # rotation
        
        # Create serial message: "TWIST,linear_x,linear_y,angular_z\n"
        serial_msg = f"TWIST,{linear_x:.3f},{linear_y:.3f},{angular_z:.3f}\n"
        
        try:
            self.serial_conn.write(serial_msg.encode('utf-8'))
            self.last_message_time = time.time()
            
            # Debug output (comment out if too verbose)
            if abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(angular_z) > 0.01:
                self.get_logger().debug(f'Sent: {serial_msg.strip()}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send serial data: {e}')
    
    def safety_check(self):
        """Send stop command if no twist messages received for too long"""
        
        timeout = 1.0  # 1 second timeout
        if time.time() - self.last_message_time > timeout:
            # Send stop command
            stop_msg = "TWIST,0.000,0.000,0.000\n"
            try:
                self.serial_conn.write(stop_msg.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Failed to send stop command: {e}')
    
    def destroy_node(self):
        """Clean up when shutting down"""
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            # Send final stop command
            stop_msg = "TWIST,0.000,0.000,0.000\n"
            self.serial_conn.write(stop_msg.encode('utf-8'))
            time.sleep(0.1)
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TwistToSerial()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
