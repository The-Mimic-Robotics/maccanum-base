#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )
    
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/ttyUSB0',
        description='ESP32 serial port'
    )
    
    # Joy node - reads Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device': LaunchConfiguration('joy_device'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Teleop twist joy - converts joy to twist messages
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[{
            'axis_linear.x': 1,      # Left stick up/down
            'axis_linear.y': 0,      # Left stick left/right  
            'axis_angular.yaw': 3,   # Right stick left/right
            'scale_linear': 1.0,
            'scale_angular': 2.0,
            'enable_button': 4,      # LB button to enable
            'enable_turbo_button': 5, # RB button for turbo
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    # Our custom twist to serial bridge
    twist_to_serial_node = Node(
        package='mecanum_robot',  # Change this to your actual package name
        executable='twist_to_serial.py',
        name='twist_to_serial',
        parameters=[{
            'serial_port': LaunchConfiguration('esp32_port'),
            'baud_rate': 115200,
            'twist_topic': '/cmd_vel',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        joy_device_arg,
        esp32_port_arg,
        joy_node,
        teleop_node,
        twist_to_serial_node,
    ])
