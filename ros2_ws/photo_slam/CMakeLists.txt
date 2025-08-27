cmake_minimum_required(VERSION 3.8)
project(photo_slam)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(std_srvs REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find PyTorch
find_package(Torch REQUIRED)

# Photo-SLAM specific paths
set(PHOTO_SLAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
set(ORB_SLAM3_ROOT "${PHOTO_SLAM_ROOT}/ORB-SLAM3")

# Include directories
include_directories(
  ${PHOTO_SLAM_ROOT}/include
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${PHOTO_SLAM_ROOT}/third_party
)

# Find ORB-SLAM3 library
find_library(ORB_SLAM3_LIBRARY
  NAMES ORB_SLAM3
  PATHS ${ORB_SLAM3_ROOT}/lib
  NO_DEFAULT_PATH
)

# Find Photo-SLAM libraries
find_library(PHOTO_SLAM_LIBRARY
  NAMES photo_slam
  PATHS ${PHOTO_SLAM_ROOT}/lib
  NO_DEFAULT_PATH
)

# Component dependencies
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  cv_bridge
  image_transport
  camera_info_manager
  std_srvs
)

# RGB-D SLAM Node
add_executable(rgbd_slam_node
  src/rgbd_slam_node.cpp
  src/photo_slam_node_base.cpp
)

target_link_libraries(rgbd_slam_node
  ${ORB_SLAM3_LIBRARY}
  ${PHOTO_SLAM_LIBRARY}
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

ament_target_dependencies(rgbd_slam_node ${dependencies})

# Stereo SLAM Node
add_executable(stereo_slam_node
  src/stereo_slam_node.cpp
  src/photo_slam_node_base.cpp
)

target_link_libraries(stereo_slam_node
  ${ORB_SLAM3_LIBRARY}
  ${PHOTO_SLAM_LIBRARY}
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

ament_target_dependencies(stereo_slam_node ${dependencies})

# Viewer Node (optional)
add_executable(gaussian_viewer_node
  src/gaussian_viewer_node.cpp
)

target_link_libraries(gaussian_viewer_node
  ${PHOTO_SLAM_LIBRARY}
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

ament_target_dependencies(gaussian_viewer_node ${dependencies})

# Install executables
install(TARGETS
  rgbd_slam_node
  stereo_slam_node
  gaussian_viewer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()