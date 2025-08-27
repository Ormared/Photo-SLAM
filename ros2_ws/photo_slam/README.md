# Photo-SLAM ROS2 Package

This package provides ROS2 nodes for running Photo-SLAM (Real-time Simultaneous Localization and Mapping with Photorealistic Gaussian Splatting) in a ROS2 environment.

## Features

- **RGB-D SLAM Node**: Processes RGB-D camera data for dense SLAM with Gaussian splatting
- **Stereo SLAM Node**: Processes stereo camera data for dense SLAM 
- **Gaussian Viewer Node**: Visualizes the Gaussian splat representation (optional)
- **ROS2 Integration**: Full integration with ROS2 ecosystem including:
  - Parameter system for configuration
  - Standard sensor message interfaces
  - TF2 transforms
  - Launch files for easy deployment
  - Service calls for trajectory saving and shutdown

## Prerequisites

- ROS2 (tested with Humble/Iron)
- Photo-SLAM dependencies:
  - ORB-SLAM3
  - PyTorch with CUDA support
  - OpenCV
  - Eigen3
- CUDA-enabled GPU (recommended)

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
# This package should already be in your Photo-SLAM repository at ros2_ws/src/photo_slam
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select photo_slam
source install/setup.bash
```

## Usage

### RGB-D SLAM

For live camera feed (e.g., RealSense D455):

```bash
ros2 launch photo_slam rgbd_slam.launch.py \
    vocabulary_path:=/path/to/ORBvoc.txt \
    orb_slam_config_path:=/path/to/realsense_d455_rgbd.yaml \
    gaussian_config_path:=/path/to/realsense_rgbd.yaml \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw
```

With configuration file:
```bash
ros2 launch photo_slam rgbd_slam.launch.py \
    --ros-args --params-file config/rgbd_example.yaml
```

### Stereo SLAM

For stereo cameras or EuRoC dataset:

```bash
ros2 launch photo_slam stereo_slam.launch.py \
    vocabulary_path:=/path/to/ORBvoc.txt \
    orb_slam_config_path:=/path/to/EuRoC.yaml \
    gaussian_config_path:=/path/to/EuRoC.yaml \
    left_topic:=/cam0/image_raw \
    right_topic:=/cam1/image_raw
```

With configuration file:
```bash
ros2 launch photo_slam stereo_slam.launch.py \
    --ros-args --params-file config/stereo_example.yaml
```

## Published Topics

- `/pose` (geometry_msgs/PoseStamped): Current camera pose
- `/path` (nav_msgs/Path): Camera trajectory
- `/odom` (nav_msgs/Odometry): Odometry information (optional)
- `/tf` and `/tf_static`: Transform tree including map→camera_link

## Services

- `/save_trajectory` (std_srvs/Trigger): Save trajectory to files
- `/shutdown` (std_srvs/Trigger): Gracefully shutdown the SLAM system

## Parameters

### Common Parameters

- `vocabulary_path` (string): Path to ORB-SLAM3 vocabulary file
- `orb_slam_config_path` (string): Path to ORB-SLAM3 configuration file  
- `gaussian_config_path` (string): Path to Gaussian mapping configuration file
- `output_directory` (string): Directory to save results (default: "/tmp/photo_slam_output")
- `map_frame_id` (string): Map frame ID (default: "map")
- `camera_frame_id` (string): Camera frame ID (default: "camera_link")
- `use_viewer` (bool): Enable Gaussian viewer (default: false)
- `publish_tf` (bool): Publish TF transforms (default: true)
- `publish_pose` (bool): Publish pose messages (default: true)
- `publish_path` (bool): Publish path messages (default: true)
- `publish_odom` (bool): Publish odometry messages (default: false)

### RGB-D Specific Parameters

- `rgb_topic` (string): RGB image topic (default: "/camera/color/image_raw")
- `depth_topic` (string): Depth image topic (default: "/camera/depth/image_rect_raw")
- `camera_info_topic` (string): Camera info topic (default: "/camera/color/camera_info")

### Stereo Specific Parameters

- `left_topic` (string): Left camera image topic (default: "/camera/left/image_raw")
- `right_topic` (string): Right camera image topic (default: "/camera/right/image_raw")
- `camera_info_topic` (string): Camera info topic (default: "/camera/left/camera_info")
- `simulate_real_time` (bool): Simulate real-time processing for datasets (default: false)

## Configuration Files

Example configuration files are provided in the `config/` directory:

- `rgbd_example.yaml`: RGB-D configuration for RealSense cameras
- `stereo_example.yaml`: Stereo configuration for EuRoC-style datasets

Edit these files to match your specific setup and camera calibration.

## Integration with Datasets

### EuRoC Dataset

```bash
# Play EuRoC dataset
ros2 bag play MH_01_easy.bag

# Run stereo SLAM
ros2 launch photo_slam stereo_slam.launch.py \
    vocabulary_path:=/path/to/ORBvoc.txt \
    orb_slam_config_path:=/path/to/EuRoC.yaml \
    gaussian_config_path:=/path/to/EuRoC.yaml \
    simulate_real_time:=true
```

### TUM RGB-D Dataset

```bash
# Convert TUM format to ROS2 bag (requires additional tools)
# Then run RGB-D SLAM
ros2 launch photo_slam rgbd_slam.launch.py \
    vocabulary_path:=/path/to/ORBvoc.txt \
    orb_slam_config_path:=/path/to/TUM.yaml \
    gaussian_config_path:=/path/to/TUM.yaml
```

## Troubleshooting

1. **CUDA out of memory**: Reduce image resolution or Gaussian mapping parameters
2. **Tracking lost**: Check camera calibration and lighting conditions
3. **Build errors**: Ensure all Photo-SLAM dependencies are properly installed
4. **No pose output**: Verify camera topics are publishing and camera info is available

## Performance Tips

- Use CUDA-enabled GPU for best performance
- Adjust `queue_size` parameter based on your system's processing speed
- Enable `simulate_real_time` for dataset playback to avoid overwhelming the system
- Consider disabling viewer (`use_viewer: false`) for better performance

## License

This package follows the same license as Photo-SLAM (GPL-3.0).