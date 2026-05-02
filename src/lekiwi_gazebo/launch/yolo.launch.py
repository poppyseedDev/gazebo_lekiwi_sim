#!/usr/bin/env python3
"""Run YOLOv8 (nano) against the sim camera.

Wraps yolo_bringup's yolo.launch.py with our defaults:
  - input topic remapped to /camera/image (our sim camera)
  - YOLOv8n model (nano, CPU-friendly; auto-downloads from Ultralytics on
    first run, then cached in ~/.config/Ultralytics)
  - device cpu (no GPU passthrough in UTM)
  - 3D detection off (we have no depth camera; we'll project to ground
    plane in our own goal-publisher node later)

Outputs:
  /yolo/detections — yolo_msgs/DetectionArray (boxes, classes, scores)
  /yolo/dbg_image  — annotated debug image (view in rqt_image_view)

Pre-req: sim already running (sim_bringup.launch.py with the camera bridge).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='yolov8n.pt',
                              description='YOLO model file (auto-downloads on first run)'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image',
                              description='Topic to subscribe for input images'),
        DeclareLaunchArgument('threshold', default_value='0.25',
                              description='Detection confidence threshold (lower = more tentative '
                                          'detections; sim banana is small + abstract so we go below '
                                          'the typical 0.4 default)'),
        DeclareLaunchArgument('device', default_value='cpu',
                              description='cpu or cuda:0'),
        DeclareLaunchArgument('imgsz_width', default_value='192',
                              description='YOLO inference width (smaller = faster on CPU). '
                                          '192 keeps a 20 cm ball detectable at ≤3 m on a 320-wide source.'),
        DeclareLaunchArgument('imgsz_height', default_value='192',
                              description='YOLO inference height'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('yolo_bringup'), 'launch', 'yolo.launch.py'
            ])),
            launch_arguments={
                'model':              LaunchConfiguration('model'),
                'input_image_topic':  LaunchConfiguration('image_topic'),
                'threshold':          LaunchConfiguration('threshold'),
                'device':             LaunchConfiguration('device'),
                'imgsz_width':        LaunchConfiguration('imgsz_width'),
                'imgsz_height':       LaunchConfiguration('imgsz_height'),
                'use_sim_time':       LaunchConfiguration('use_sim_time'),
                'use_3d':             'False',
                'use_tracking':       'False',
            }.items(),
        ),
    ])
