#!/usr/bin/env python3
"""Launch Nav2 against a saved map.

Wraps nav2_bringup's bringup_launch.py with our omni-drive params.
Pair with lekiwi_gazebo/sim_bringup.launch.py (sim) or with the real-robot
bringup (real). Pass map:=<path/to/map.yaml> to load a specific map.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_navigation'), 'maps', 'empty.yaml'
            ]),
            description='Path to map.yaml. Ignored when use_localization:=False '
                        '(active SLAM provides the map dynamically).',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_navigation'), 'config', 'nav2_params.yaml'
            ]),
            description='Nav2 parameters file (omni-drive tuned)',
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Whether to run all nodes in a single component container',
        ),
        DeclareLaunchArgument(
            'use_localization', default_value='True',
            description='Bring up amcl + map_server. Set False for active SLAM '
                        '(slam_toolbox supplies map → odom TF and /map externally).',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'bringup_launch.py',
            ])),
            launch_arguments={
                'use_sim_time':      LaunchConfiguration('use_sim_time'),
                'map':               LaunchConfiguration('map'),
                'params_file':       LaunchConfiguration('params_file'),
                'autostart':         LaunchConfiguration('autostart'),
                'use_composition':   LaunchConfiguration('use_composition'),
                'use_localization':  LaunchConfiguration('use_localization'),
            }.items(),
        ),
    ])
