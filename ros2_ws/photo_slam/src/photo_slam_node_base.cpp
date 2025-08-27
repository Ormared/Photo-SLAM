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

#include "photo_slam_node_base.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace photo_slam_ros
{

PhotoSlamNodeBase::PhotoSlamNodeBase(const std::string& node_name)
    : Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Photo-SLAM node: %s", node_name.c_str());
    
    // Declare parameters
    declareParameters();
    
    // Setup publishers and services
    setupPublishers();
    setupServices();
    
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Setup path message
    path_msg_.header.frame_id = map_frame_id_;
}

PhotoSlamNodeBase::~PhotoSlamNodeBase()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Photo-SLAM node");
    
    shutdown_requested_ = true;
    
    if (slam_system_) {
        slam_system_->Shutdown();
    }
    
    if (gaussian_mapping_thread_.joinable()) {
        gaussian_mapping_thread_.join();
    }
    
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void PhotoSlamNodeBase::declareParameters()
{
    // File paths
    this->declare_parameter("vocabulary_path", std::string(""));
    this->declare_parameter("orb_slam_config_path", std::string(""));
    this->declare_parameter("gaussian_config_path", std::string(""));
    this->declare_parameter("output_directory", std::string("/tmp/photo_slam_output"));
    
    // Frame IDs
    this->declare_parameter("map_frame_id", std::string("map"));
    this->declare_parameter("camera_frame_id", std::string("camera"));
    
    // Feature flags
    this->declare_parameter("use_viewer", false);
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("publish_pose", true);
    this->declare_parameter("publish_path", true);
    this->declare_parameter("publish_odom", false);
    
    // Get parameters
    vocabulary_path_ = this->get_parameter("vocabulary_path").as_string();
    orb_slam_config_path_ = this->get_parameter("orb_slam_config_path").as_string();
    gaussian_config_path_ = this->get_parameter("gaussian_config_path").as_string();
    output_directory_ = this->get_parameter("output_directory").as_string();
    map_frame_id_ = this->get_parameter("map_frame_id").as_string();
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    use_viewer_ = this->get_parameter("use_viewer").as_bool();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    publish_pose_ = this->get_parameter("publish_pose").as_bool();
    publish_path_ = this->get_parameter("publish_path").as_bool();
    publish_odom_ = this->get_parameter("publish_odom").as_bool();
    
    // Create output directory
    std::filesystem::path output_dir(output_directory_);
    std::filesystem::create_directories(output_dir);
    
    // Device configuration
    if (torch::cuda::is_available()) {
        RCLCPP_INFO(this->get_logger(), "CUDA available! Training on GPU.");
        device_type_ = torch::kCUDA;
    } else {
        RCLCPP_INFO(this->get_logger(), "Training on CPU.");
        device_type_ = torch::kCPU;
    }
}

void PhotoSlamNodeBase::setupPublishers()
{
    if (publish_pose_) {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    }
    
    if (publish_path_) {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    }
    
    if (publish_odom_) {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    }
}

void PhotoSlamNodeBase::setupServices()
{
    save_trajectory_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "save_trajectory",
        std::bind(&PhotoSlamNodeBase::saveTrajectoryCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    shutdown_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "shutdown",
        std::bind(&PhotoSlamNodeBase::shutdownCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
}

void PhotoSlamNodeBase::initializePhotoSlam()
{
    RCLCPP_INFO(this->get_logger(), "Creating ORB-SLAM3 system...");
    
    // Note: SLAM mode should be set in derived classes based on sensor type
    // This is a virtual method that should be overridden
}

void PhotoSlamNodeBase::publishPose(const cv::Mat& Tcw, const rclcpp::Time& timestamp, const std::string& frame_id)
{
    if (!publish_pose_ || !pose_pub_) return;
    
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.pose = matrixToPose(Tcw.inv()); // Convert from camera-to-world to world-to-camera
    
    pose_pub_->publish(pose_msg);
}

void PhotoSlamNodeBase::publishPath(const cv::Mat& Tcw, const rclcpp::Time& timestamp, const std::string& frame_id)
{
    if (!publish_path_ || !path_pub_) return;
    
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = timestamp;
    pose_stamped.header.frame_id = map_frame_id_;
    pose_stamped.pose = matrixToPose(Tcw.inv());
    
    path_msg_.header.stamp = timestamp;
    path_msg_.poses.push_back(pose_stamped);
    
    path_pub_->publish(path_msg_);
}

void PhotoSlamNodeBase::publishOdometry(const cv::Mat& Tcw, const rclcpp::Time& timestamp, const std::string& frame_id)
{
    if (!publish_odom_ || !odom_pub_) return;
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = map_frame_id_;
    odom_msg.child_frame_id = camera_frame_id_;
    odom_msg.pose.pose = matrixToPose(Tcw.inv());
    
    // Note: Velocity estimation could be added here by tracking previous poses
    
    odom_pub_->publish(odom_msg);
}

void PhotoSlamNodeBase::publishTransform(const cv::Mat& Tcw, const rclcpp::Time& timestamp,
                                        const std::string& frame_id, const std::string& child_frame_id)
{
    if (!publish_tf_) return;
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = map_frame_id_;
    transform_stamped.child_frame_id = camera_frame_id_;
    
    geometry_msgs::msg::Pose pose = matrixToPose(Tcw.inv());
    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;
    transform_stamped.transform.rotation = pose.orientation;
    
    tf_broadcaster_->sendTransform(transform_stamped);
}

geometry_msgs::msg::Pose PhotoSlamNodeBase::matrixToPose(const cv::Mat& T)
{
    geometry_msgs::msg::Pose pose;
    
    // Extract translation
    pose.position.x = T.at<float>(0, 3);
    pose.position.y = T.at<float>(1, 3);
    pose.position.z = T.at<float>(2, 3);
    
    // Extract rotation matrix and convert to quaternion
    cv::Mat R = T.rowRange(0, 3).colRange(0, 3);
    
    // Convert rotation matrix to quaternion
    double trace = R.at<float>(0, 0) + R.at<float>(1, 1) + R.at<float>(2, 2);
    if (trace > 0.0) {
        double s = sqrt(trace + 1.0);
        pose.orientation.w = s * 0.5;
        s = 0.5 / s;
        pose.orientation.x = (R.at<float>(2, 1) - R.at<float>(1, 2)) * s;
        pose.orientation.y = (R.at<float>(0, 2) - R.at<float>(2, 0)) * s;
        pose.orientation.z = (R.at<float>(1, 0) - R.at<float>(0, 1)) * s;
    } else {
        int i = R.at<float>(0, 0) < R.at<float>(1, 1) ? (R.at<float>(1, 1) < R.at<float>(2, 2) ? 2 : 1) : (R.at<float>(0, 0) < R.at<float>(2, 2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        
        double s = sqrt(R.at<float>(i, i) - R.at<float>(j, j) - R.at<float>(k, k) + 1.0);
        double q[4];
        q[i] = s * 0.5;
        s = 0.5 / s;
        q[3] = (R.at<float>(k, j) - R.at<float>(j, k)) * s;
        q[j] = (R.at<float>(j, i) + R.at<float>(i, j)) * s;
        q[k] = (R.at<float>(k, i) + R.at<float>(i, k)) * s;
        
        pose.orientation.x = q[0];
        pose.orientation.y = q[1];
        pose.orientation.z = q[2];
        pose.orientation.w = q[3];
    }
    
    return pose;
}

void PhotoSlamNodeBase::saveTrajectoryCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    try {
        std::filesystem::path output_dir(output_directory_);
        
        if (slam_system_) {
            slam_system_->SaveTrajectoryTUM((output_dir / "CameraTrajectory_TUM.txt").string());
            slam_system_->SaveKeyFrameTrajectoryTUM((output_dir / "KeyFrameTrajectory_TUM.txt").string());
            slam_system_->SaveTrajectoryEuRoC((output_dir / "CameraTrajectory_EuRoC.txt").string());
            slam_system_->SaveKeyFrameTrajectoryEuRoC((output_dir / "KeyFrameTrajectory_EuRoC.txt").string());
            slam_system_->SaveTrajectoryKITTI((output_dir / "CameraTrajectory_KITTI.txt").string());
        }
        
        // Save tracking time statistics
        if (!tracking_times_.empty()) {
            std::ofstream out((output_dir / "TrackingTime.txt").string());
            for (const auto& time : tracking_times_) {
                out << std::fixed << std::setprecision(4) << time << std::endl;
            }
            out.close();
        }
        
        // Save GPU peak memory usage
        saveGpuPeakMemoryUsage(output_dir / "GpuPeakUsageMB.txt");
        
        response->success = true;
        response->message = "Trajectory and statistics saved to " + output_directory_;
        RCLCPP_INFO(this->get_logger(), "Trajectory saved successfully");
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Failed to save trajectory: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to save trajectory: %s", e.what());
    }
}

void PhotoSlamNodeBase::shutdownCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Shutdown requested via service");
    shutdown_requested_ = true;
    
    response->success = true;
    response->message = "Shutdown initiated";
    
    // Trigger node shutdown
    rclcpp::shutdown();
}

void PhotoSlamNodeBase::saveGpuPeakMemoryUsage(const std::filesystem::path& pathSave)
{
    try {
        namespace c10Alloc = c10::cuda::CUDACachingAllocator;
        c10Alloc::DeviceStats mem_stats = c10Alloc::getDeviceStats(0);
        
        c10Alloc::Stat reserved_bytes = mem_stats.reserved_bytes[static_cast<int>(c10Alloc::StatType::AGGREGATE)];
        float max_reserved_MB = reserved_bytes.peak / (1024.0 * 1024.0);
        
        c10Alloc::Stat alloc_bytes = mem_stats.allocated_bytes[static_cast<int>(c10Alloc::StatType::AGGREGATE)];
        float max_alloc_MB = alloc_bytes.peak / (1024.0 * 1024.0);
        
        std::ofstream out(pathSave);
        out << "Peak reserved (MB): " << max_reserved_MB << std::endl;
        out << "Peak allocated (MB): " << max_alloc_MB << std::endl;
        out.close();
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to save GPU memory usage: %s", e.what());
    }
}

} // namespace photo_slam_ros