import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The Joystick Node (Reads the Controller)
        Node(
            package='rov_joystick',
            executable='joy_node',
            output='screen'
        ),
        
        # 2. The Text Bridge (Converts Joystick -> 'f', 'b', 'u' for ESP32)
        # We use this INSTEAD of thruster_controller + serial_bridge
        Node(
            package='rov_control',
            executable='joy_to_text',
            output='screen'
        )
    ])