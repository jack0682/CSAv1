#include <GL/glew.h>
#include <GL/gl.h>
#include <pangolin/pangolin.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>

// ORB-SLAM2
#include "System.h"

class SlamPoseNode : public rclcpp::Node
{
public:
  SlamPoseNode()
  : Node("slam_pose_node")
  {
    // Declare & get parameters
    this->declare_parameter<std::string>("vocab_path", "/home/jack/ORB_SLAM2/Vocabulary/ORBvoc.txt");
    this->declare_parameter<std::string>("config_path", "/home/jack/ORB_SLAM2/Examples/RGB-D/TUM1.yaml");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("max_time_diff", 0.5);  // 🔑 최대 허용 시간 차 파라미터

    this->get_parameter("vocab_path", vocab_path_);
    this->get_parameter("config_path", config_path_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("max_time_diff", max_time_diff_);

    slam_ = std::make_shared<ORB_SLAM2::System>(
      vocab_path_, config_path_, ORB_SLAM2::System::RGBD, true
    );

    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera/pose", 10);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", qos_profile,
      std::bind(&SlamPoseNode::image_callback, this, std::placeholders::_1)
    );

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", qos_profile,
      std::bind(&SlamPoseNode::depth_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "🔧 ORB-SLAM2 RGB-D 노드가 시작되었습니다 (허용 Δt: %.3fs)", max_time_diff_);
  }

private:
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    try {
      latest_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
      latest_depth_time_ = msg->header.stamp;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge depth 에러: %s", e.what());
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat image;
    try {
      image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(this->get_logger(), "cv_bridge RGB 에러: %s", e.what());
      return;
    }

    cv::Mat depth;
    rclcpp::Time stamp = msg->header.stamp;
    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      double dt = std::abs((stamp - latest_depth_time_).seconds());
      if (dt > max_time_diff_) {
        RCLCPP_WARN(this->get_logger(), 
          "🕒 Depth frame과 RGB frame Δt=%.5fs > 허용 %.3fs → depth 생략.",
          dt, max_time_diff_);
        return;
      }
      depth = latest_depth_.clone();
    }

    double timestamp = stamp.seconds();

    cv::Mat Tcw = slam_->TrackRGBD(image, depth, timestamp);

    if (!Tcw.empty()) {
      publish_pose(Tcw, stamp);
    }
  }

  void publish_pose(const cv::Mat& Tcw, const rclcpp::Time& stamp)
  {
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc * tcw;

    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    for (int i = 0; i < 3; ++i) {
      t(i) = twc.at<float>(i);
      for (int j = 0; j < 3; ++j)
        R(i,j) = Rwc.at<float>(i,j);
    }

    Eigen::Quaternionf q(R);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.pose.position.x = t(0);
    pose_msg.pose.position.y = t(1);
    pose_msg.pose.position.z = t(2);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    publisher_->publish(pose_msg);
  }

  std::shared_ptr<ORB_SLAM2::System> slam_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  std::string vocab_path_;
  std::string config_path_;
  std::string frame_id_;
  double max_time_diff_;  // 🔑

  cv::Mat latest_depth_;
  rclcpp::Time latest_depth_time_;
  std::mutex depth_mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
