#!/usr/bin/env python3

"""
Convert Replica dataset to ROS2 bag format using rosbags library.

This script converts the Replica office0 dataset (RGB images, depth images, and trajectory)
into a ROS2 bag that can be used with the Photo-SLAM ROS2 nodes.

Usage:
    python3 replica_to_rosbag2.py --dataset_path data/Replica/office0 \
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
import time
from typing import List, Tuple, Dict, Any

# rosbags imports
from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr


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


def create_camera_info_msg(camera_params: Dict[str, Any], timestamp: Dict, frame_id: str) -> Dict:
    """Create a CameraInfo message from camera parameters."""
    camera_info = {
        'header': {
            'stamp': timestamp,
            'frame_id': frame_id
        },
        'width': camera_params['width'],
        'height': camera_params['height'],
        'distortion_model': "plumb_bob",
        'd': [
            camera_params['k1'], camera_params['k2'], camera_params['p1'], 
            camera_params['p2'], camera_params['k3']
        ],
        'k': [
            camera_params['fx'], 0.0, camera_params['cx'],
            0.0, camera_params['fy'], camera_params['cy'],
            0.0, 0.0, 1.0
        ],
        'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'p': [
            camera_params['fx'], 0.0, camera_params['cx'], 0.0,
            0.0, camera_params['fy'], camera_params['cy'], 0.0,
            0.0, 0.0, 1.0, 0.0
        ],
        'binning_x': 0,
        'binning_y': 0,
        'roi': {
            'x_offset': 0,
            'y_offset': 0,
            'height': 0,
            'width': 0,
            'do_rectify': False
        }
    }
    
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


def create_image_msg(image: np.ndarray, timestamp: Dict, frame_id: str, encoding: str) -> Dict:
    """Create an Image message dictionary from a numpy array."""
    height, width = image.shape[:2]
    
    if encoding == 'rgb8':
        step = width * 3
        data = image.flatten().tobytes()
    elif encoding == '16UC1':
        step = width * 2
        data = image.astype(np.uint16).tobytes()
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    
    return {
        'header': {
            'stamp': timestamp,
            'frame_id': frame_id
        },
        'height': height,
        'width': width,
        'encoding': encoding,
        'is_bigendian': False,
        'step': step,
        'data': data
    }


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert rotation matrix to quaternion (x, y, z, w)."""
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    
    return qx, qy, qz, qw


def convert_to_rosbag(dataset_path: str, config_path: str, output_bag: str):
    """Convert Replica dataset to ROS2 bag."""
    
    # Load camera parameters and trajectory
    camera_params = load_camera_params(config_path)
    trajectory = load_trajectory(os.path.join(dataset_path, 'traj.txt'))
    rgb_files, depth_files = get_image_files(dataset_path)
    
    # Create output directory
    Path(output_bag).parent.mkdir(parents=True, exist_ok=True)
    
    # Calculate time step (assuming constant framerate)
    dt = 1.0 / camera_params['fps']  # seconds per frame
    
    # Create path message
    path_poses = []
    
    print("Converting dataset to ROS2 bag...")
    print(f"Processing {len(rgb_files)} frames...")
    
    # Use rosbags Writer
    with Writer(output_bag) as writer:
        # Create topic connections
        rgb_connection = writer.add_connection(
            '/camera/color/image_raw',
            'sensor_msgs/msg/Image'
        )
        depth_connection = writer.add_connection(
            '/camera/depth/image_rect_raw', 
            'sensor_msgs/msg/Image'
        )
        camera_info_connection = writer.add_connection(
            '/camera/color/camera_info',
            'sensor_msgs/msg/CameraInfo'
        )
        path_connection = writer.add_connection(
            '/trajectory_path',
            'nav_msgs/msg/Path'
        )
        
        for i, (rgb_file, depth_file) in enumerate(zip(rgb_files, depth_files)):
            if i % 100 == 0:
                print(f"Processing frame {i}/{len(rgb_files)}")
            
            # Calculate timestamp
            timestamp_sec = i * dt
            timestamp_nsec = int((timestamp_sec % 1) * 1e9)
            timestamp_sec = int(timestamp_sec)
            
            ros_timestamp = {
                'sec': timestamp_sec,
                'nanosec': timestamp_nsec
            }
            
            # Load and convert RGB image
            rgb_image = cv2.imread(rgb_file)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
            rgb_msg = create_image_msg(rgb_image, ros_timestamp, 'camera_color_optical_frame', 'rgb8')
            
            depth_image = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)
            depth_msg = create_image_msg(depth_image, ros_timestamp, 'camera_depth_optical_frame', '16UC1')
            
            camera_info = create_camera_info_msg(camera_params, ros_timestamp, 'camera_color_optical_frame')
            
            if i < len(trajectory):
                pose_matrix = trajectory[i]
                
                translation = pose_matrix[:3, 3]
                
                rotation_matrix = pose_matrix[:3, :3]
                qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)
                
                pose_stamped = {
                    'header': {
                        'stamp': ros_timestamp,
                        'frame_id': "map"
                    },
                    'pose': {
                        'position': {
                            'x': float(translation[0]),
                            'y': float(translation[1]),
                            'z': float(translation[2])
                        },
                        'orientation': {
                            'x': qx,
                            'y': qy,
                            'z': qz,
                            'w': qw
                        }
                    }
                }
                
                path_poses.append(pose_stamped)
            
            timestamp_ns = timestamp_sec * int(1e9) + timestamp_nsec
            
            writer.write(rgb_connection, timestamp_ns, serialize_cdr(rgb_msg, 'sensor_msgs/msg/Image'))
            writer.write(depth_connection, timestamp_ns, serialize_cdr(depth_msg, 'sensor_msgs/msg/Image'))
            writer.write(camera_info_connection, timestamp_ns, serialize_cdr(camera_info, 'sensor_msgs/msg/CameraInfo'))
        
        final_timestamp_ns = len(rgb_files) * int(dt * 1e9)
        final_timestamp = {
            'sec': int(final_timestamp_ns // 1e9),
            'nanosec': int(final_timestamp_ns % 1e9)
        }
        
        path_msg = {
            'header': {
                'stamp': final_timestamp,
                'frame_id': "map"
            },
            'poses': path_poses
        }
        
        writer.write(path_connection, final_timestamp_ns, serialize_cdr(path_msg, 'nav_msgs/msg/Path'))
    
    print(f"Successfully created ROS2 bag: {output_bag}")
    print(f"Topics created:")
    print(f"  - /camera/color/image_raw: {len(rgb_files)} messages")
    print(f"  - /camera/depth/image_rect_raw: {len(depth_files)} messages") 
    print(f"  - /camera/color/camera_info: {len(rgb_files)} messages")
    print(f"  - /trajectory_path: 1 message with {len(path_poses)} poses")


def main():
    parser = argparse.ArgumentParser(description='Convert Replica dataset to ROS2 bag using rosbags library')
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
    
    convert_to_rosbag(args.dataset_path, args.config_path, args.output_bag)


if __name__ == '__main__':
    main()