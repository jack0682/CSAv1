#ifndef CSA_SLAM_INTERFACE__SLAM_POSE_NODE_HPP_
#define CSA_SLAM_INTERFACE__SLAM_POSE_NODE_HPP_

// ROS2 core
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// ROS2 utilities
#include "cv_bridge/cv_bridge.h"

// ORB-SLAM2 core
#include "System.h"

// External libraries
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief ROS2 Node that wraps ORB-SLAM2 for RGB-D input.
 */
class SlamPoseNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor that initializes the ORB-SLAM2 system and ROS2 interfaces.
   */
  SlamPoseNode();

private:
  /**
   * @brief Callback for processing incoming RGB images and publishing poses.
   * @param msg The ROS2 Image message.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // ORB-SLAM2 system instance
  std::shared_ptr<ORB_SLAM2::System> slam_;

  // ROS2 interfaces
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Node parameters
  std::string vocab_path_;
  std::string config_path_;
  std::string frame_id_;
};

#endif  // CSA_SLAM_INTERFACE__SLAM_POSE_NODE_HPP_
