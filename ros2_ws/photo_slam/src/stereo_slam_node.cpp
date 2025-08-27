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

class StereoSlamNode : public PhotoSlamNodeBase
{
public:
    StereoSlamNode();
    ~StereoSlamNode() = default;

private:
    void initializePhotoSlam() override;
    void setupSubscribers();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
    
    // ROS2 subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
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
    
    // Timing for realistic playback
    rclcpp::Time last_frame_time_;
    bool simulate_real_time_;
};

StereoSlamNode::StereoSlamNode()
    : PhotoSlamNodeBase("stereo_slam_node"),
      camera_info_received_(false),
      frame_counter_(0),
      image_scale_(1.0f),
      simulate_real_time_(false)
{
    RCLCPP_INFO(this->get_logger(), "Starting Stereo SLAM Node");
    
    // Additional parameters for stereo
    this->declare_parameter("left_topic", std::string("/camera/left/image_raw"));
    this->declare_parameter("right_topic", std::string("/camera/right/image_raw"));
    this->declare_parameter("camera_info_topic", std::string("/camera/left/camera_info"));
    this->declare_parameter("queue_size", 10);
    this->declare_parameter("simulate_real_time", false);
    
    simulate_real_time_ = this->get_parameter("simulate_real_time").as_bool();
    
    setupSubscribers();
    initializePhotoSlam();
}

void StereoSlamNode::setupSubscribers()
{
    std::string left_topic = this->get_parameter("left_topic").as_string();
    std::string right_topic = this->get_parameter("right_topic").as_string();
    std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
    int queue_size = this->get_parameter("queue_size").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Subscribing to:");
    RCLCPP_INFO(this->get_logger(), "  Left: %s", left_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Right: %s", right_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera Info: %s", camera_info_topic.c_str());
    
    // Setup synchronized subscribers for left and right images
    left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, left_topic);
    right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        this, right_topic);
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&StereoSlamNode::imageCallback, this,
                                     std::placeholders::_1, std::placeholders::_2));
    
    // Camera info subscriber
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&StereoSlamNode::cameraInfoCallback, this, std::placeholders::_1));
}

void StereoSlamNode::initializePhotoSlam()
{
    if (vocabulary_path_.empty() || orb_slam_config_path_.empty() || gaussian_config_path_.empty()) {
        RCLCPP_FATAL(this->get_logger(), 
                     "Required parameters not set: vocabulary_path, orb_slam_config_path, gaussian_config_path");
        rclcpp::shutdown();
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Creating ORB-SLAM3 system for Stereo...");
    slam_system_ = std::make_shared<ORB_SLAM3::System>(
        vocabulary_path_, orb_slam_config_path_, ORB_SLAM3::System::STEREO);
    
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

void StereoSlamNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
{
    if (!camera_info_received_) {
        camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera info received");
    }
}

void StereoSlamNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                                  const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
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
        // Real-time simulation logic (similar to original euroc_stereo.cpp)
        if (simulate_real_time_ && !last_frame_time_.nanoseconds() == 0) {
            rclcpp::Time current_time = left_msg->header.stamp;
            double time_diff = (current_time - last_frame_time_).seconds();
            
            // Sleep to maintain real-time playback
            if (time_diff > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(time_diff));
            }
        }
        last_frame_time_ = left_msg->header.stamp;
        
        // Convert ROS images to OpenCV
        cv_bridge::CvImageConstPtr cv_left_ptr;
        cv_bridge::CvImageConstPtr cv_right_ptr;
        
        try {
            cv_left_ptr = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Left): %s", e.what());
            return;
        }
        
        try {
            cv_right_ptr = cv_bridge::toCvShare(right_msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Right): %s", e.what());
            return;
        }
        
        cv::Mat left_image = cv_left_ptr->image.clone();
        cv::Mat right_image = cv_right_ptr->image.clone();
        
        // Apply image scaling if needed
        if (image_scale_ != 1.0f) {
            int width = left_image.cols * image_scale_;
            int height = left_image.rows * image_scale_;
            cv::resize(left_image, left_image, cv::Size(width, height));
            cv::resize(right_image, right_image, cv::Size(width, height));
        }
        
        // Get timestamp (convert nanoseconds to seconds)
        double timestamp = left_msg->header.stamp.sec + left_msg->header.stamp.nanosec * 1e-9;
        rclcpp::Time ros_time = left_msg->header.stamp;
        
        // Track with SLAM system
        auto t1 = std::chrono::steady_clock::now();
        
        cv::Mat Tcw = slam_system_->TrackStereo(left_image, right_image, timestamp, 
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
            publishPose(Tcw, ros_time, left_msg->header.frame_id);
            publishPath(Tcw, ros_time, left_msg->header.frame_id);
            publishOdometry(Tcw, ros_time, left_msg->header.frame_id);
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
        auto node = std::make_shared<photo_slam_ros::StereoSlamNode>();
        
        RCLCPP_INFO(node->get_logger(), "Stereo SLAM node started, spinning...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("stereo_slam_node"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}