#!/usr/bin/env python3
"""Active SLAM stack: sim + slam_toolbox + Nav2 + explore_lite frontier explorer.

This is the canonical autonomous-mapping pipeline:
  - Gazebo simulates the world and the robot
  - slam_toolbox builds a map from /scan and publishes map → odom TF
  - Nav2 (without map_server / amcl) plans paths and controls the robot
  - explore_lite picks frontiers and dispatches NavigateToPose goals to Nav2

Usage:
  ros2 launch lekiwi_gazebo active_slam.launch.py world:=small_house.sdf

When exploration finishes (no frontiers left), explore_lite drives the robot
back to the start. Save the resulting map with:
  ros2 run nav2_map_server map_saver_cli -f <name> --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gz   = FindPackageShare('lekiwi_gazebo')
    pkg_nav  = FindPackageShare('lekiwi_navigation')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_gz, 'launch', 'sim_bringup.launch.py'
        ])),
        launch_arguments={
            'world':    LaunchConfiguration('world'),
            'rviz':     LaunchConfiguration('rviz'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    # Delay slam_toolbox by 8s so Gazebo has finished spawning the robot and
    # parameter_bridge is forwarding /scan + /clock. If slam activates before
    # /scan starts flowing, the first scan can be dropped and slam then waits
    # for the robot to move minimum_travel_distance before publishing /map.
    slam = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                pkg_gz, 'launch', 'slam.launch.py'
            ])),
            launch_arguments={'use_sim_time': 'true'}.items(),
        )],
    )

    # Nav2 without map_server / amcl — slam_toolbox supplies map → odom TF
    # and the /map topic.
    # use_composition:=False so each Nav2 node is its own process. Composition
    # causes a startup race where bt_navigator tries to discover the 'spin'
    # action before behavior_server has come up, and the 1s timeout is too tight.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_nav, 'launch', 'nav2.launch.py'
        ])),
        launch_arguments={
            'use_sim_time':     'true',
            'use_localization': 'False',
            'use_composition':  'False',
        }.items(),
    )

    # Delay explore_lite startup so slam_toolbox has published several /map
    # updates (5s interval) AND its map→odom TF is firmly in the buffer.
    # explore_lite has a hardcoded 0.1s timeout for the initial TF lookup,
    # which is much too short on a slow VM — if it fires before TF is ready,
    # the first frontier search returns a degenerate centroid at the robot's
    # own pose and explore_lite declares "all frontiers traversed" and stops.
    # 45s is conservative; tune down on faster hosts.
    explore = TimerAction(
        period=45.0,
        actions=[Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_gz, 'config', 'explore_lite.yaml']),
                {'use_sim_time': True},
            ],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='small_house.sdf'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        sim,
        slam,
        nav2,
        explore,
    ])
