#!/usr/bin/env python3
"""
Launch file for Photo-SLAM RGB-D SLAM

Usage:
    ros2 launch photo_slam rgbd_slam.launch.py \
        vocabulary_path:=/path/to/ORBvoc.txt \
        orb_slam_config_path:=/path/to/config.yaml \
        gaussian_config_path:=/path/to/gaussian_config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


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
    
    declare_rgb_topic = DeclareLaunchArgument(
        'rgb_topic',
        description='RGB image topic',
        default_value='/camera/color/image_raw'
    )
    
    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic', 
        description='Depth image topic',
        default_value='/camera/depth/image_rect_raw'
    )
    
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        description='Camera info topic',
        default_value='/camera/color/camera_info'
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
    
    # RGB-D SLAM Node
    rgbd_slam_node = Node(
        package='photo_slam',
        executable='rgbd_slam_node',
        name='rgbd_slam_node',
        output='screen',
        parameters=[{
            'vocabulary_path': LaunchConfiguration('vocabulary_path'),
            'orb_slam_config_path': LaunchConfiguration('orb_slam_config_path'),
            'gaussian_config_path': LaunchConfiguration('gaussian_config_path'),
            'output_directory': LaunchConfiguration('output_directory'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'use_viewer': LaunchConfiguration('use_viewer'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'publish_pose': LaunchConfiguration('publish_pose'),
            'publish_path': LaunchConfiguration('publish_path'),
            'publish_odom': LaunchConfiguration('publish_odom'),
            'queue_size': 10
        }],
        remappings=[
            ('/camera/color/image_raw', LaunchConfiguration('rgb_topic')),
            ('/camera/depth/image_rect_raw', LaunchConfiguration('depth_topic')),
            ('/camera/color/camera_info', LaunchConfiguration('camera_info_topic'))
        ]
    )
    
    return LaunchDescription([
        declare_vocabulary_path,
        declare_orb_slam_config_path,
        declare_gaussian_config_path,
        declare_output_directory,
        declare_rgb_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        declare_map_frame_id,
        declare_camera_frame_id,
        declare_use_viewer,
        declare_publish_tf,
        declare_publish_pose,
        declare_publish_path,
        declare_publish_odom,
        rgbd_slam_node
    ])