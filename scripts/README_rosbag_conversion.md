# Replica Dataset to ROS2 Bag Conversion

This script converts the Replica dataset (specifically the office0 scene) into a ROS2 bag format that can be used with the Photo-SLAM ROS2 nodes.

## Features

- Converts RGB images (`frame*.jpg`) to `sensor_msgs/Image` 
- Converts depth images (`depth*.png`) to `sensor_msgs/Image`
- Creates camera info messages from ORB-SLAM3 config files
- Converts trajectory data to a path message
- Uses proper timestamps based on camera FPS
- Compatible with rosbags library (no ROS2 installation required)

## Requirements

Install the required Python packages:

```bash
pip install -r requirements_rosbag_conversion.txt
```

Or manually install:

```bash
pip install rosbags opencv-python numpy pyyaml
```

## Usage

### Basic usage:

```bash
python3 scripts/replica_to_rosbag2.py \
    --dataset_path data/Replica/office0 \
    --config_path cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
    --output_bag output_bags/replica_office0
```

### Parameters:

- `--dataset_path`: Path to the Replica dataset directory (must contain `traj.txt` and `results/` folder)
- `--config_path`: Path to the ORB-SLAM3 config file (for camera parameters)
- `--output_bag`: Output path for the ROS2 bag (directory will be created if it doesn't exist)

## Output Topics

The generated bag will contain the following topics:

- `/camera/color/image_raw` - RGB images (sensor_msgs/Image)
- `/camera/depth/image_rect_raw` - Depth images (sensor_msgs/Image)  
- `/camera/color/camera_info` - Camera calibration info (sensor_msgs/CameraInfo)
- `/trajectory_path` - Ground truth trajectory (nav_msgs/Path)

## Dataset Structure Expected

```
data/Replica/office0/
├── traj.txt                    # Trajectory file (4x4 transformation matrices)
└── results/
    ├── frame000000.jpg         # RGB images
    ├── frame000001.jpg
    ├── ...
    ├── depth000000.png         # Depth images
    ├── depth000001.png
    └── ...
```

## Camera Frame IDs

- RGB images: `camera_color_optical_frame`
- Depth images: `camera_depth_optical_frame`  
- Trajectory: `map` frame

## Usage with Photo-SLAM

Once the bag is created, you can use it with the Photo-SLAM ROS2 nodes:

```bash
# Source ROS2 and Photo-SLAM workspace
source /opt/ros/humble/setup.bash
cd ros2_ws && source install/setup.bash

# Play the bag
ros2 bag play output_bags/replica_office0

# In another terminal, run Photo-SLAM
ros2 launch photo_slam rgbd_slam.launch.py \
    vocabulary_path:=ORB-SLAM3/Vocabulary/ORBvoc.txt \
    orb_slam_config_path:=cfg/ORB_SLAM3/RGB-D/Replica/office0.yaml \
    gaussian_config_path:=cfg/gaussian_mapper/RGB-D/Replica/office0.yaml
```

## Notes

- The script assumes a constant framerate as specified in the config file
- Trajectory data is converted from 4x4 transformation matrices to ROS poses
- The conversion preserves the original image encodings (RGB8 for color, 16UC1 for depth)
- Timestamps start from 0 and increment based on the camera FPS