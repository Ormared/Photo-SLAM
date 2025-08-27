#!/usr/bin/env python3
"""
Launch file for Photo-SLAM Stereo SLAM

Usage:
    ros2 launch photo_slam stereo_slam.launch.py \
        vocabulary_path:=/path/to/ORBvoc.txt \
        orb_slam_config_path:=/path/to/config.yaml \
        gaussian_config_path:=/path/to/gaussian_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_share = FindPackageShare('photo_slam')
    
    # Launch arguments
    declare_vocabulary_path = DeclareLaunchArgument(
        'vocabulary_path',
        description='Path to ORB-SLAM3 vocabulary file',
        default_value=''
    )
    
    declare_orb_slam_config_path = DeclareLaunchArgument(
        'orb_slam_config_path',
        description='Path to ORB-SLAM3 configuration file',
        default_value=''
    )
    
    declare_gaussian_config_path = DeclareLaunchArgument(
        'gaussian_config_path',
        description='Path to Gaussian mapping configuration file',
        default_value=''
    )
    
    declare_output_directory = DeclareLaunchArgument(
        'output_directory',
        description='Directory to save results',
        default_value='/tmp/photo_slam_output'
    )
    
    declare_left_topic = DeclareLaunchArgument(
        'left_topic',
        description='Left camera image topic',
        default_value='/camera/left/image_raw'
    )
    
    declare_right_topic = DeclareLaunchArgument(
        'right_topic',
        description='Right camera image topic',
        default_value='/camera/right/image_raw'
    )
    
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        description='Camera info topic',
        default_value='/camera/left/camera_info'
    )
    
    declare_map_frame_id = DeclareLaunchArgument(
        'map_frame_id',
        description='Map frame ID',
        default_value='map'
    )
    
    declare_camera_frame_id = DeclareLaunchArgument(
        'camera_frame_id',
        description='Camera frame ID',
        default_value='camera_link'
    )
    
    declare_use_viewer = DeclareLaunchArgument(
        'use_viewer',
        description='Enable Gaussian viewer',
        default_value='false'
    )
    
    declare_simulate_real_time = DeclareLaunchArgument(
        'simulate_real_time',
        description='Simulate real-time processing (useful for datasets)',
        default_value='false'
    )
    
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        description='Publish TF transforms',
        default_value='true'
    )
    
    declare_publish_pose = DeclareLaunchArgument(
        'publish_pose',
        description='Publish pose messages',
        default_value='true'
    )
    
    declare_publish_path = DeclareLaunchArgument(
        'publish_path',
        description='Publish path messages',
        default_value='true'
    )
    
    declare_publish_odom = DeclareLaunchArgument(
        'publish_odom',
        description='Publish odometry messages',
        default_value='false'
    )
    
    # Stereo SLAM Node
    stereo_slam_node = Node(
        package='photo_slam',
        executable='stereo_slam_node',
        name='stereo_slam_node',
        output='screen',
        parameters=[{
            'vocabulary_path': LaunchConfiguration('vocabulary_path'),
            'orb_slam_config_path': LaunchConfiguration('orb_slam_config_path'),
            'gaussian_config_path': LaunchConfiguration('gaussian_config_path'),
            'output_directory': LaunchConfiguration('output_directory'),
            'left_topic': LaunchConfiguration('left_topic'),
            'right_topic': LaunchConfiguration('right_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'use_viewer': LaunchConfiguration('use_viewer'),
            'simulate_real_time': LaunchConfiguration('simulate_real_time'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_pose': LaunchConfiguration('publish_pose'),
            'publish_path': LaunchConfiguration('publish_path'),
            'publish_odom': LaunchConfiguration('publish_odom'),
            'queue_size': 10
        }],
        remappings=[
            ('/camera/left/image_raw', LaunchConfiguration('left_topic')),
            ('/camera/right/image_raw', LaunchConfiguration('right_topic')),
            ('/camera/left/camera_info', LaunchConfiguration('camera_info_topic'))
        ]
    )
    
    return LaunchDescription([
        declare_vocabulary_path,
        declare_orb_slam_config_path,
        declare_gaussian_config_path,
        declare_output_directory,
        declare_left_topic,
        declare_right_topic,
        declare_camera_info_topic,
        declare_map_frame_id,
        declare_camera_frame_id,
        declare_use_viewer,
        declare_simulate_real_time,
        declare_publish_tf,
        declare_publish_pose,
        declare_publish_path,
        declare_publish_odom,
        stereo_slam_node
    ])