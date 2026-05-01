#!/usr/bin/env python3
"""Bring up LeKiwi in Gazebo Harmonic with ROS 2 bridge.

Starts:
  - gz sim (the empty world with a few boxes for SLAM)
  - robot_state_publisher fed by lekiwi.sim.urdf.xacro (use_sim_time:=true)
  - the spawner that creates the robot in the world from /robot_description
  - parameter_bridge wiring /clock, /cmd_vel, /scan, /odom, /tf
  - optional RViz with the description package's display config
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    world      = LaunchConfiguration('world').perform(context)
    use_rviz   = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    headless   = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_x    = LaunchConfiguration('x').perform(context)
    robot_y    = LaunchConfiguration('y').perform(context)
    robot_z    = LaunchConfiguration('z').perform(context)
    render     = LaunchConfiguration('render_engine').perform(context)

    pkg_gz   = FindPackageShare('lekiwi_gazebo').perform(context)
    pkg_desc = FindPackageShare('lekiwi_description').perform(context)
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim').perform(context)

    # Tell gz-sim where to find package:// → model:// meshes. The URDF parser
    # rewrites package://lekiwi_description/... to model://lekiwi_description/...
    # which is then resolved against GZ_SIM_RESOURCE_PATH. Pointing at the
    # parent of <pkg>/share/lekiwi_description lets that path resolve.
    share_dir = os.path.dirname(pkg_desc)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = (
        f'{share_dir}:{existing_resource_path}' if existing_resource_path else share_dir
    )
    xacro = FindExecutable(name='xacro').perform(context)

    urdf_xacro = f'{pkg_gz}/urdf/lekiwi.sim.urdf.xacro'
    world_path = f'{pkg_gz}/worlds/{world}'
    bridge_yaml = f'{pkg_gz}/config/ros_gz_bridge.yaml'

    robot_description = {
        'robot_description': ParameterValue(
            Command([f'{xacro} {urdf_xacro}']), value_type=str
        ),
        'use_sim_time': True,
    }

    gz_args = f'-r -v 4 {world_path}'
    if headless:
        gz_args = f'-s -r -v 4 {world_path}'
    if render:
        gz_args = f'--render-engine {render} ' + gz_args

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'lekiwi',
            '-x', robot_x,
            '-y', robot_y,
            '-z', robot_z,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
        output='screen',
    )

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gz_sim,
        rsp,
        bridge,
        spawn,
    ]

    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', f'{pkg_desc}/rviz/display.rviz'],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='World file in lekiwi_gazebo/worlds/'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz alongside the sim'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz sim without GUI'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.06'),
        DeclareLaunchArgument('render_engine', default_value='ogre',
                              description='gz render engine. Default ogre (OGRE1) is much faster '
                                          'on UTM/Apple Silicon where EGL falls back to swrast. '
                                          'Pass render_engine:=ogre2 for nicer visuals on real GPUs.'),
        OpaqueFunction(function=launch_setup),
    ])
