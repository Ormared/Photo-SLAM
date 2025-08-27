#!/usr/bin/env python3

"""
Convert Replica dataset to ROS2 bag format for Photo-SLAM.

This script converts the Replica office0 dataset (RGB images, depth images, and trajectory)
into a ROS2 bag that can be used with the Photo-SLAM ROS2 nodes.

Usage:
    python3 convert_replica_to_rosbag.py --dataset_path data/Replica/office0 \
                                         --config_path cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
                                         --output_bag output_bags/replica_office0
"""

import argparse
import os
import sys
import cv2
import numpy as np
import yaml
from pathlib import Path
import sqlite3
import time
from typing import List, Tuple, Dict, Any

# ROS2 imports
import rclpy
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from builtin_interfaces.msg import Time as TimeMsg

def load_camera_params(config_path: str) -> Dict[str, Any]:
    """Load camera parameters from ORB-SLAM3 config file."""
    print(f"Loading camera parameters from: {config_path}")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    camera_params = {
        'width': config['Camera.width'],
        'height': config['Camera.height'],
        'fps': config['Camera.fps'],
        'fx': config['Camera1.fx'],
        'fy': config['Camera1.fy'],
        'cx': config['Camera1.cx'],
        'cy': config['Camera1.cy'],
        'k1': config['Camera1.k1'],
        'k2': config['Camera1.k2'],
        'k3': config['Camera1.k3'],
        'p1': config['Camera1.p1'],
        'p2': config['Camera1.p2'],
        'depth_factor': config['RGBD.DepthMapFactor']
    }
    
    print(f"Camera resolution: {camera_params['width']}x{camera_params['height']}")
    print(f"Camera FPS: {camera_params['fps']}")
    print(f"Depth factor: {camera_params['depth_factor']}")
    
    return camera_params

def create_camera_info_msg(camera_params: Dict[str, Any], timestamp: TimeMsg, frame_id: str) -> CameraInfo:
    """Create a CameraInfo message from camera parameters."""
    camera_info = CameraInfo()
    
    # Header
    camera_info.header.stamp = timestamp
    camera_info.header.frame_id = frame_id
    
    # Image dimensions
    camera_info.width = camera_params['width']
    camera_info.height = camera_params['height']
    
    # Distortion model
    camera_info.distortion_model = "plumb_bob"
    camera_info.d = [
        camera_params['k1'], camera_params['k2'], camera_params['p1'], 
        camera_params['p2'], camera_params['k3']
    ]
    
    # Camera matrix (K)
    camera_info.k = [
        camera_params['fx'], 0.0, camera_params['cx'],
        0.0, camera_params['fy'], camera_params['cy'],
        0.0, 0.0, 1.0
    ]
    
    # Rectification matrix (identity for monocular)
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    
    # Projection matrix
    camera_info.p = [
        camera_params['fx'], 0.0, camera_params['cx'], 0.0,
        0.0, camera_params['fy'], camera_params['cy'], 0.0,
        0.0, 0.0, 1.0, 0.0
    ]
    
    return camera_info

def load_trajectory(traj_path: str) -> List[np.ndarray]:
    """Load trajectory data from traj.txt file."""
    print(f"Loading trajectory from: {traj_path}")
    
    poses = []
    with open(traj_path, 'r') as f:
        for line in f:
            # Each line contains 16 values representing a 4x4 transformation matrix
            values = [float(x) for x in line.strip().split()]
            if len(values) == 16:
                # Reshape to 4x4 matrix (row-major order)
                pose_matrix = np.array(values).reshape(4, 4)
                poses.append(pose_matrix)
    
    print(f"Loaded {len(poses)} poses")
    return poses

def get_image_files(dataset_path: str) -> Tuple[List[str], List[str]]:
    """Get sorted lists of RGB and depth image files."""
    results_path = os.path.join(dataset_path, 'results')
    
    # Get all RGB frames
    rgb_files = []
    depth_files = []
    
    for i in range(2000):  # We know there are 2000 frames
        rgb_file = os.path.join(results_path, f'frame{i:06d}.jpg')
        depth_file = os.path.join(results_path, f'depth{i:06d}.png')
        
        if os.path.exists(rgb_file) and os.path.exists(depth_file):
            rgb_files.append(rgb_file)
            depth_files.append(depth_file)
    
    print(f"Found {len(rgb_files)} RGB-D pairs")
    return rgb_files, depth_files

def convert_to_rosbag(dataset_path: str, config_path: str, output_bag: str):
    """Convert Replica dataset to ROS2 bag."""
    
    # Load camera parameters and trajectory
    camera_params = load_camera_params(config_path)
    trajectory = load_trajectory(os.path.join(dataset_path, 'traj.txt'))
    rgb_files, depth_files = get_image_files(dataset_path)
    
    # Create output directory
    Path(output_bag).parent.mkdir(parents=True, exist_ok=True)
    
    # Initialize rosbag2 writer
    storage_options = rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)
    
    # Create topics
    rgb_topic_info = rosbag2_py.TopicMetadata(
        name='/camera/color/image_raw',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    )
    
    depth_topic_info = rosbag2_py.TopicMetadata(
        name='/camera/depth/image_rect_raw',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    )
    
    camera_info_topic_info = rosbag2_py.TopicMetadata(
        name='/camera/color/camera_info',
        type='sensor_msgs/msg/CameraInfo',
        serialization_format='cdr'
    )
    
    path_topic_info = rosbag2_py.TopicMetadata(
        name='/trajectory_path',
        type='nav_msgs/msg/Path',
        serialization_format='cdr'
    )
    
    writer.create_topic(rgb_topic_info)
    writer.create_topic(depth_topic_info)
    writer.create_topic(camera_info_topic_info)
    writer.create_topic(path_topic_info)
    
    # Initialize CV bridge
    bridge = CvBridge()
    
    # Calculate time step (assuming constant framerate)
    dt = 1.0 / camera_params['fps']  # seconds per frame
    
    # Create path message
    path_msg = Path()
    path_msg.header.frame_id = "map"
    
    print("Converting dataset to ROS2 bag...")
    print(f"Processing {len(rgb_files)} frames...")
    
    for i, (rgb_file, depth_file) in enumerate(zip(rgb_files, depth_files)):
        if i % 100 == 0:
            print(f"Processing frame {i}/{len(rgb_files)}")
        
        # Calculate timestamp
        timestamp_sec = i * dt
        timestamp_nsec = int((timestamp_sec % 1) * 1e9)
        timestamp_sec = int(timestamp_sec)
        
        ros_timestamp = TimeMsg()
        ros_timestamp.sec = timestamp_sec
        ros_timestamp.nanosec = timestamp_nsec
        
        # Load and convert RGB image
        rgb_image = cv2.imread(rgb_file)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        rgb_msg = bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
        rgb_msg.header.stamp = ros_timestamp
        rgb_msg.header.frame_id = 'camera_color_optical_frame'
        
        # Load and convert depth image
        depth_image = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)
        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        depth_msg.header.stamp = ros_timestamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        
        # Create camera info
        camera_info = create_camera_info_msg(camera_params, ros_timestamp, 'camera_color_optical_frame')
        
        # Add pose to path (if trajectory data is available)
        if i < len(trajectory):
            pose_matrix = trajectory[i]
            
            # Extract translation
            translation = pose_matrix[:3, 3]
            
            # Extract rotation matrix and convert to quaternion
            rotation_matrix = pose_matrix[:3, :3]
            
            # Convert rotation matrix to quaternion using Rodrigues rotation
            # This is a simplified conversion - for production use, consider using scipy or similar
            trace = np.trace(rotation_matrix)
            if trace > 0:
                s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
                qw = 0.25 * s
                qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            else:
                if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                    s = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
                    qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                    qx = 0.25 * s
                    qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                    qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                    s = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
                    qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                    qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                    qy = 0.25 * s
                    qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                else:
                    s = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
                    qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                    qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                    qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                    qz = 0.25 * s
            
            # Create pose stamped message
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = ros_timestamp
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position = Point(x=float(translation[0]), y=float(translation[1]), z=float(translation[2]))
            pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            
            path_msg.poses.append(pose_stamped)
        
        # Write messages to bag
        timestamp_ns = timestamp_sec * int(1e9) + timestamp_nsec
        
        writer.write('/camera/color/image_raw', serialize_message(rgb_msg), timestamp_ns)
        writer.write('/camera/depth/image_rect_raw', serialize_message(depth_msg), timestamp_ns)
        writer.write('/camera/color/camera_info', serialize_message(camera_info), timestamp_ns)
    
    # Write path message once at the end
    final_timestamp_ns = len(rgb_files) * int(dt * 1e9)
    path_msg.header.stamp = TimeMsg()
    path_msg.header.stamp.sec = int(final_timestamp_ns // 1e9)
    path_msg.header.stamp.nanosec = int(final_timestamp_ns % 1e9)
    
    writer.write('/trajectory_path', serialize_message(path_msg), final_timestamp_ns)
    
    print(f"Successfully created ROS2 bag: {output_bag}")
    print(f"Topics created:")
    print(f"  - /camera/color/image_raw: {len(rgb_files)} messages")
    print(f"  - /camera/depth/image_rect_raw: {len(depth_files)} messages") 
    print(f"  - /camera/color/camera_info: {len(rgb_files)} messages")
    print(f"  - /trajectory_path: 1 message with {len(path_msg.poses)} poses")

def main():
    parser = argparse.ArgumentParser(description='Convert Replica dataset to ROS2 bag')
    parser.add_argument('--dataset_path', required=True, help='Path to Replica dataset directory')
    parser.add_argument('--config_path', required=True, help='Path to ORB-SLAM3 config file')
    parser.add_argument('--output_bag', required=True, help='Output ROS2 bag path')
    
    args = parser.parse_args()
    
    # Validate input paths
    if not os.path.exists(args.dataset_path):
        print(f"Error: Dataset path does not exist: {args.dataset_path}")
        sys.exit(1)
    
    if not os.path.exists(args.config_path):
        print(f"Error: Config path does not exist: {args.config_path}")
        sys.exit(1)
    
    # Check if trajectory file exists
    traj_path = os.path.join(args.dataset_path, 'traj.txt')
    if not os.path.exists(traj_path):
        print(f"Error: Trajectory file not found: {traj_path}")
        sys.exit(1)
    
    # Check if results directory exists
    results_path = os.path.join(args.dataset_path, 'results')
    if not os.path.exists(results_path):
        print(f"Error: Results directory not found: {results_path}")
        sys.exit(1)
    
    print("Starting conversion...")
    print(f"Dataset path: {args.dataset_path}")
    print(f"Config path: {args.config_path}")
    print(f"Output bag: {args.output_bag}")
    
    # Initialize ROS2 (required for message serialization)
    rclpy.init()
    
    try:
        convert_to_rosbag(args.dataset_path, args.config_path, args.output_bag)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()