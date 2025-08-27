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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <thread>

#include "viewer/imgui_viewer.h"

namespace photo_slam_ros
{

class GaussianViewerNode : public rclcpp::Node
{
public:
    GaussianViewerNode();
    ~GaussianViewerNode() = default;

private:
    void setupSubscribers();
    void mapDataCallback(const std_msgs::msg::String::ConstSharedPtr& msg);
    
    // Viewer
    std::shared_ptr<ImGuiViewer> viewer_;
    std::thread viewer_thread_;
    
    // ROS2 components
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_data_sub_;
    
    // Parameters
    std::string map_data_topic_;
    
    // Shutdown flag
    std::atomic<bool> shutdown_requested_{false};
};

GaussianViewerNode::GaussianViewerNode()
    : Node("gaussian_viewer_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting Gaussian Viewer Node");
    
    // Parameters
    this->declare_parameter("map_data_topic", std::string("/gaussian_map_data"));
    map_data_topic_ = this->get_parameter("map_data_topic").as_string();
    
    setupSubscribers();
    
    // Note: This is a simplified viewer node
    // In a full implementation, you would need to integrate with the actual
    // Gaussian mapping data structures and visualization system
    RCLCPP_INFO(this->get_logger(), "Gaussian viewer initialized");
}

void GaussianViewerNode::setupSubscribers()
{
    RCLCPP_INFO(this->get_logger(), "Subscribing to map data topic: %s", map_data_topic_.c_str());
    
    map_data_sub_ = this->create_subscription<std_msgs::msg::String>(
        map_data_topic_, 10,
        std::bind(&GaussianViewerNode::mapDataCallback, this, std::placeholders::_1));
}

void GaussianViewerNode::mapDataCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    if (shutdown_requested_) {
        return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received map data update");
    
    // In a full implementation, this would process Gaussian splat data
    // and update the visualization accordingly
}

} // namespace photo_slam_ros

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<photo_slam_ros::GaussianViewerNode>();
        
        RCLCPP_INFO(node->get_logger(), "Gaussian viewer node started, spinning...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("gaussian_viewer_node"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}