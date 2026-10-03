#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class LidarToPx4Bridge : public rclcpp::Node {
public:
  LidarToPx4Bridge() : Node("lidar_to_px4_bridge") {
    this->declare_parameter<std::string>("real_robot_odom_topic", "/aft_mapped_to_init");
    this->declare_parameter<std::string>("vision_pose_topic", "/mavros/vision_pose/pose");
    this->declare_parameter<bool>("reset_origin_on_start", true);
    const auto odom_topic = this->get_parameter("real_robot_odom_topic").as_string();
    const auto vision_pose_topic = this->get_parameter("vision_pose_topic").as_string();
    reset_origin_on_start_ = this->get_parameter("reset_origin_on_start").as_bool();

    // 订阅 PointLIO 输出
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        LidarToPx4Bridge::odomCallback(msg);
      });
    RCLCPP_INFO(this->get_logger(), "订阅实机odom: %s", odom_topic.c_str());

    // 订阅 Fastlio 输出
    //odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/absolute_pose", 10, std::bind(&LidarToPx4Bridge::odomCallback, this, std::placeholders::_1));

    // 发布给 PX4 飞控
    vision_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(vision_pose_topic, 10);
    RCLCPP_INFO(this->get_logger(), "发布PX4 vision pose: %s", vision_pose_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "PX4 vision pose首帧归零: %s",
      reset_origin_on_start_ ? "true" : "false");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& pose = msg->pose.pose;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf2::Vector3 p(pose.position.x, pose.position.y, pose.position.z);

    if (!received_first_odom_) {
      received_first_odom_ = true;
      origin_position_ = p;
      origin_orientation_ = q;
      origin_orientation_inv_ = origin_orientation_.inverse();
      RCLCPP_INFO(
        this->get_logger(),
        "收到首帧odom: frame_id=%s child_frame_id=%s raw=(%.3f, %.3f, %.3f)",
        msg->header.frame_id.c_str(),
        msg->child_frame_id.c_str(),
        pose.position.x,
        pose.position.y,
        pose.position.z);
    }

    auto vision_pose = geometry_msgs::msg::PoseStamped();
    // 设置时间戳
    vision_pose.header.stamp = this->now();
    vision_pose.header.frame_id = "map";

    tf2::Vector3 out_position = p;
    tf2::Quaternion out_orientation = q;
    if (reset_origin_on_start_) {
      out_position = tf2::quatRotate(origin_orientation_inv_, p - origin_position_);
      out_orientation = origin_orientation_inv_ * q;
      out_orientation.normalize();
    }

    // 位置
    vision_pose.pose.position.x = out_position.x();
    vision_pose.pose.position.y = out_position.y();
    vision_pose.pose.position.z = out_position.z();

    // 姿态（四元数）
    vision_pose.pose.orientation = tf2::toMsg(out_orientation);

    vision_pose_pub->publish(vision_pose);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "已发布vision pose: x=%.3f y=%.3f z=%.3f",
      vision_pose.pose.position.x,
      vision_pose.pose.position.y,
      vision_pose.pose.position.z);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub;
  bool received_first_odom_{false};
  bool reset_origin_on_start_{true};
  tf2::Vector3 origin_position_{0.0, 0.0, 0.0};
  tf2::Quaternion origin_orientation_{0.0, 0.0, 0.0, 1.0};
  tf2::Quaternion origin_orientation_inv_{0.0, 0.0, 0.0, 1.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarToPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}
