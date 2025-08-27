/**
 * This file is part of Photo-SLAM
 *
 * Copyright (C) 2023-2024 Longwei Li and Hui Cheng, Sun Yat-sen University.
 * Copyright (C) 2023-2024 Huajian Huang and Sai-Kit Yeung, Hong Kong University of Science and Technology.
 *
 * Photo-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Photo-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Photo-SLAM.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PHOTO_SLAM_NODE_BASE_HPP
#define PHOTO_SLAM_NODE_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <filesystem>

#include "ORB-SLAM3/include/System.h"
#include "include/gaussian_mapper.h"
#include "viewer/imgui_viewer.h"

namespace photo_slam_ros
{

class PhotoSlamNodeBase : public rclcpp::Node
{
public:
    PhotoSlamNodeBase(const std::string& node_name);
    virtual ~PhotoSlamNodeBase();

protected:
    // Common initialization
    virtual void initializePhotoSlam();
    virtual void declareParameters();
    
    // Publishers
    void setupPublishers();
    void publishPose(const cv::Mat& Tcw, const rclcpp::Time& timestamp, const std::string& frame_id);
    void publishPath(const cv::Mat& Tcw, const rclcpp::Time& timestamp, const std::string& frame_id);
    void publishOdometry(const cv::Mat& Tcw, const rclcpp::Time& timestamp, const std::string& frame_id);
    void publishTransform(const cv::Mat& Tcw, const rclcpp::Time& timestamp, 
                         const std::string& frame_id, const std::string& child_frame_id);

    // Services
    void setupServices();
    void saveTrajectoryCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void shutdownCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Utility functions
    cv::Mat convertTcwToMatrix(const cv::Mat& Tcw);
    geometry_msgs::msg::Pose matrixToPose(const cv::Mat& T);
    void saveGpuPeakMemoryUsage(const std::filesystem::path& pathSave);

    // Core SLAM components
    std::shared_ptr<ORB_SLAM3::System> slam_system_;
    std::shared_ptr<GaussianMapper> gaussian_mapper_;
    std::shared_ptr<ImGuiViewer> viewer_;
    
    // Threading
    std::thread gaussian_mapping_thread_;
    std::thread viewer_thread_;
    
    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_trajectory_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Path tracking
    nav_msgs::msg::Path path_msg_;
    
    // Parameters
    std::string vocabulary_path_;
    std::string orb_slam_config_path_;
    std::string gaussian_config_path_;
    std::string output_directory_;
    std::string map_frame_id_;
    std::string camera_frame_id_;
    bool use_viewer_;
    bool publish_tf_;
    bool publish_pose_;
    bool publish_path_;
    bool publish_odom_;
    
    // Device configuration
    torch::DeviceType device_type_;
    
    // Tracking statistics
    std::vector<float> tracking_times_;
    std::mutex tracking_mutex_;
    
    // Shutdown flag
    std::atomic<bool> shutdown_requested_{false};
};

} // namespace photo_slam_ros

#endif // PHOTO_SLAM_NODE_BASE_HPP