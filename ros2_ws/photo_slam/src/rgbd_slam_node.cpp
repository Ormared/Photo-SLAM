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
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

namespace photo_slam_ros
{

class RgbdSlamNode : public PhotoSlamNodeBase
{
public:
    RgbdSlamNode();
    ~RgbdSlamNode() = default;

private:
    void initializePhotoSlam() override;
    void setupSubscribers();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
    
    // ROS2 subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // Message synchronization
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Camera parameters
    bool camera_info_received_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    
    // Frame counter
    size_t frame_counter_;
    
    // Image scaling
    float image_scale_;
};

RgbdSlamNode::RgbdSlamNode()
    : PhotoSlamNodeBase("rgbd_slam_node"),
      camera_info_received_(false),
      frame_counter_(0),
      image_scale_(1.0f)
{
    RCLCPP_INFO(this->get_logger(), "Starting RGB-D SLAM Node");
    
    // Additional parameters for RGB-D
    this->declare_parameter("rgb_topic", std::string("/camera/color/image_raw"));
    this->declare_parameter("depth_topic", std::string("/camera/depth/image_rect_raw"));
    this->declare_parameter("camera_info_topic", std::string("/camera/color/camera_info"));
    this->declare_parameter("queue_size", 10);
    
    setupSubscribers();
    initializePhotoSlam();
}

void RgbdSlamNode::setupSubscribers()
{
    std::string rgb_topic = this->get_parameter("rgb_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    int queue_size = this->get_parameter("queue_size").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to:");
    RCLCPP_INFO(this->get_logger(), "  RGB: %s", rgb_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Depth: %s", depth_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera Info: %s", camera_info_topic.c_str());
    
    // Setup synchronized subscribers for RGB and depth images
    rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, rgb_topic);
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, depth_topic);
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), *rgb_sub_, *depth_sub_);
    sync_->registerCallback(std::bind(&RgbdSlamNode::imageCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));
    
    // Camera info subscriber
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&RgbdSlamNode::cameraInfoCallback, this, std::placeholders::_1));
}

void RgbdSlamNode::initializePhotoSlam()
{
    if (vocabulary_path_.empty() || orb_slam_config_path_.empty() || gaussian_config_path_.empty()) {
        RCLCPP_FATAL(this->get_logger(), 
                     "Required parameters not set: vocabulary_path, orb_slam_config_path, gaussian_config_path");
        rclcpp::shutdown();
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Creating ORB-SLAM3 system for RGB-D...");
    slam_system_ = std::make_shared<ORB_SLAM3::System>(
        vocabulary_path_, orb_slam_config_path_, ORB_SLAM3::System::RGBD);
    
    image_scale_ = slam_system_->GetImageScale();
    RCLCPP_INFO(this->get_logger(), "Image scale factor: %.2f", image_scale_);
    
    // Create GaussianMapper
    std::filesystem::path gaussian_cfg_path(gaussian_config_path_);
    std::filesystem::path output_dir(output_directory_);
    
    RCLCPP_INFO(this->get_logger(), "Creating GaussianMapper...");
    gaussian_mapper_ = std::make_shared<GaussianMapper>(
        slam_system_, gaussian_cfg_path, output_dir, 0, device_type_);
    
    // Start Gaussian mapping thread
    gaussian_mapping_thread_ = std::thread(&GaussianMapper::run, gaussian_mapper_.get());
    
    // Create viewer if enabled
    if (use_viewer_) {
        RCLCPP_INFO(this->get_logger(), "Creating viewer...");
        viewer_ = std::make_shared<ImGuiViewer>(slam_system_, gaussian_mapper_);
        viewer_thread_ = std::thread(&ImGuiViewer::run, viewer_.get());
    }
    
    RCLCPP_INFO(this->get_logger(), "Photo-SLAM system initialized successfully");
}

void RgbdSlamNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
{
    if (!camera_info_received_) {
        camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received");
    }
}

void RgbdSlamNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    if (shutdown_requested_) {
        return;
    }
    
    if (!slam_system_) {
        RCLCPP_WARN(this->get_logger(), "SLAM system not initialized yet");
        return;
    }
    
    if (slam_system_->isShutDown()) {
        RCLCPP_WARN(this->get_logger(), "SLAM system has been shut down");
        return;
    }
    
    try {
        // Convert ROS images to OpenCV
        cv_bridge::CvImageConstPtr cv_rgb_ptr;
        cv_bridge::CvImageConstPtr cv_depth_ptr;
        
        try {
            cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
            return;
        }
        
        try {
            cv_depth_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Depth): %s", e.what());
            return;
        }
        
        cv::Mat rgb_image = cv_rgb_ptr->image.clone();
        cv::Mat depth_image = cv_depth_ptr->image.clone();
        
        // Convert BGR to RGB for ORB-SLAM3
        cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB);
        
        // Apply image scaling if needed
        if (image_scale_ != 1.0f) {
            int width = rgb_image.cols * image_scale_;
            int height = rgb_image.rows * image_scale_;
            cv::resize(rgb_image, rgb_image, cv::Size(width, height));
            cv::resize(depth_image, depth_image, cv::Size(width, height));
        }
        
        // Get timestamp
        double timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;
        rclcpp::Time ros_time = rgb_msg->header.stamp;
        
        // Track with SLAM system
        auto t1 = std::chrono::steady_clock::now();
        
        cv::Mat Tcw = slam_system_->TrackRGBD(rgb_image, depth_image, timestamp, 
                                             std::vector<ORB_SLAM3::IMU::Point>(), 
                                             "frame_" + std::to_string(frame_counter_));
        
        auto t2 = std::chrono::steady_clock::now();
        double track_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        
        // Store tracking time
        {
            std::lock_guard<std::mutex> lock(tracking_mutex_);
            tracking_times_.push_back(track_time);
        }
        
        // Check if tracking was successful
        if (!Tcw.empty()) {
            // Publish pose, path, odometry, and tf
            publishPose(Tcw, ros_time, rgb_msg->header.frame_id);
            publishPath(Tcw, ros_time, rgb_msg->header.frame_id);
            publishOdometry(Tcw, ros_time, rgb_msg->header.frame_id);
            publishTransform(Tcw, ros_time, map_frame_id_, camera_frame_id_);
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "Frame %zu processed successfully (tracking time: %.4f s)", 
                        frame_counter_, track_time);
        } else {
            RCLCPP_WARN(this->get_logger(), "Tracking lost at frame %zu", frame_counter_);
        }
        
        frame_counter_++;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in image callback: %s", e.what());
    }
}

} // namespace photo_slam_ros

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<photo_slam_ros::RgbdSlamNode>();
        
        RCLCPP_INFO(node->get_logger(), "RGB-D SLAM node started, spinning...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("rgbd_slam_node"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}