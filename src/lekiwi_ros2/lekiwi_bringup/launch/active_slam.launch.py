#!/usr/bin/env python3
"""Active SLAM on the real LeKiwi (Raspberry Pi 5).

The hardware-side equivalent of lekiwi_gazebo/launch/active_slam.launch.py:
  - lekiwi_bringup/launch/lekiwi.launch.py  → STS3215 motor drivers,
    BNO055 IMU, LD06 LiDAR, OAK-D
  - lekiwi_navigation/launch/nav.launch.py  → robot_localization EKF
    (wheel odom + IMU → /odom + odom→base_footprint TF)
  - slam_toolbox (use_sim_time:=false)      → /map + map→odom TF
  - Nav2 (use_localization:=False)          → planner + MPPI controller
  - explore_lite                             → frontier explorer

Prereqs on the Pi:
  - This workspace cloned + built
  - The two real-only drivers cloned into src/ (skipped in sim):
      git clone https://github.com/ldrobotSensorTeam/ldlidar_ros2 src/ldlidar_ros2
      git clone https://github.com/flynneva/bno055 src/bno055
      rosdep install --from-paths src -y --ignore-src
      colcon build --symlink-install
  - The STS3215 driver:
      git clone https://github.com/adityakamath/sts_hardware_interface src/sts_hardware_interface

Usage on the Pi:
  ros2 launch lekiwi_bringup active_slam.launch.py

To map manually instead of with explore_lite, omit the explore node and drive
with teleop_twist_keyboard (running on your laptop, with `ROS_DOMAIN_ID`
matched to the Pi).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare('lekiwi_bringup')
    pkg_nav     = FindPackageShare('lekiwi_navigation')
    pkg_gz      = FindPackageShare('lekiwi_gazebo')   # hosts slam + explore configs

    # Real-robot bringup: motor controller, IMU, laser, webcam
    # config:=base brings up base + IMU + laser + webcam (skips pantilt + OAK-D
    # to keep the mapping run lightweight).
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_bringup, 'launch', 'lekiwi.launch.py'
        ])),
        launch_arguments={
            'config':      'base',
            'diagnostics': 'true',
        }.items(),
    )

    # slam_toolbox with our omni-tuned config; use_sim_time forced false
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_gz, 'launch', 'slam.launch.py'
        ])),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # Nav2 navigation only — slam_toolbox supplies /map + map→odom TF;
    # the EKF (started by lekiwi.launch.py via nav.launch.py) supplies
    # odom→base_footprint.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_nav, 'launch', 'nav2.launch.py'
        ])),
        launch_arguments={
            'use_sim_time':     'false',
            'use_localization': 'False',
        }.items(),
    )

    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_gz, 'config', 'explore_lite.yaml']),
            {'use_sim_time': False},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('explore', default_value='true',
                              description='Run explore_lite. Set false to drive manually with teleop.'),
        bringup,
        slam,
        nav2,
        explore,
    ])
