#!/usr/bin/env python3
"""Visualize the LeKiwi URDF in RViz with optional joint sliders.

Spins up robot_state_publisher fed by xacro, joint_state_publisher_gui for
interactive wheel/pantilt joint control, and RViz with a config that shows
the robot model and TF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    config   = LaunchConfiguration('config').perform(context)
    use_gui  = LaunchConfiguration('gui').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'

    pkg_desc = FindPackageShare('lekiwi_description').perform(context)
    xacro    = FindExecutable(name='xacro').perform(context)

    if config == 'base':
        urdf = f'{pkg_desc}/urdf/base/base.urdf.xacro'
    elif config == 'pantilt':
        urdf = f'{pkg_desc}/urdf/pantilt/pantilt.urdf.xacro'
    else:
        urdf = f'{pkg_desc}/urdf/lekiwi.urdf.xacro'

    xacro_cmd = f'{xacro} {urdf} use_mock:=true'

    robot_description = {
        'robot_description': ParameterValue(Command([xacro_cmd]), value_type=str)
    }

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
    ]

    if use_gui:
        actions.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ))
    else:
        actions.append(Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ))

    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', f'{pkg_desc}/rviz/display.rviz'],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='lekiwi',
            description='URDF to display: base, pantilt, or lekiwi',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use joint_state_publisher_gui (sliders) instead of joint_state_publisher',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz with the display config',
        ),
        OpaqueFunction(function=launch_setup),
    ])
