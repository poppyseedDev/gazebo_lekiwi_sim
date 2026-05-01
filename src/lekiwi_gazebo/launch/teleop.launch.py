#!/usr/bin/env python3
"""Spawn teleop_twist_keyboard in a new terminal so it gets stdin focus."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[('/cmd_vel', '/cmd_vel')],
        ),
    ])
