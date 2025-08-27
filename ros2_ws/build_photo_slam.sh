#!/bin/bash

# Build script for Photo-SLAM ROS2 package
# This script helps build the Photo-SLAM ROS2 integration

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Photo-SLAM ROS2 Build Script ===${NC}"

# Check if we're in the right directory
if [ ! -f "src/photo_slam/package.xml" ]; then
    echo -e "${RED}Error: Please run this script from the ros2_ws directory${NC}"
    echo "Expected directory structure:"
    echo "  ros2_ws/"
    echo "    ├── src/"
    echo "    │   └── photo_slam/"
    echo "    │       ├── package.xml"
    echo "    │       └── ..."
    echo "    └── build_photo_slam.sh"
    exit 1
fi

# Check ROS2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS_DISTRO not set. Attempting to source ROS2...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}Sourced ROS2 Humble${NC}"
    elif [ -f "/opt/ros/iron/setup.bash" ]; then
        source /opt/ros/iron/setup.bash
        echo -e "${GREEN}Sourced ROS2 Iron${NC}"
    else
        echo -e "${RED}Error: Could not find ROS2 installation${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}Using ROS2 distribution: $ROS_DISTRO${NC}"

# Install dependencies
echo -e "${GREEN}Installing dependencies...${NC}"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Check for required Photo-SLAM components
PHOTO_SLAM_ROOT="../.."
if [ ! -d "$PHOTO_SLAM_ROOT/ORB-SLAM3" ]; then
    echo -e "${YELLOW}Warning: ORB-SLAM3 directory not found at $PHOTO_SLAM_ROOT/ORB-SLAM3${NC}"
    echo "Make sure ORB-SLAM3 is built and available"
fi

if [ ! -d "$PHOTO_SLAM_ROOT/include" ]; then
    echo -e "${YELLOW}Warning: Photo-SLAM include directory not found at $PHOTO_SLAM_ROOT/include${NC}"
    echo "Make sure Photo-SLAM is built and available"
fi

# Build the package
echo -e "${GREEN}Building photo_slam package...${NC}"
colcon build --packages-select photo_slam --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check build result
if [ $? -eq 0 ]; then
    echo -e "${GREEN}=== Build successful! ===${NC}"
    echo -e "${GREEN}To use the package, run:${NC}"
    echo "  source install/setup.bash"
    echo ""
    echo -e "${GREEN}Example usage:${NC}"
    echo "  # RGB-D SLAM"
    echo "  ros2 launch photo_slam rgbd_slam.launch.py \\"
    echo "      vocabulary_path:=/path/to/ORBvoc.txt \\"
    echo "      orb_slam_config_path:=/path/to/config.yaml \\"
    echo "      gaussian_config_path:=/path/to/gaussian_config.yaml"
    echo ""
    echo "  # Stereo SLAM"
    echo "  ros2 launch photo_slam stereo_slam.launch.py \\"
    echo "      vocabulary_path:=/path/to/ORBvoc.txt \\"
    echo "      orb_slam_config_path:=/path/to/config.yaml \\"
    echo "      gaussian_config_path:=/path/to/gaussian_config.yaml"
    echo ""
    echo -e "${GREEN}See src/photo_slam/README.md for detailed documentation${NC}"
else
    echo -e "${RED}=== Build failed! ===${NC}"
    echo "Check the error messages above for details"
    exit 1
fi