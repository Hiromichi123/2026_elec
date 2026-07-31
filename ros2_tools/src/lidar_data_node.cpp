/*
 * 功能:
 * 订阅里程计数据（实机或仿真），处理后发布雷达位姿信息
 * 兼容PointLIO里程计消息格式
 * 发布消息类型: ros2_tools::msg::LidarPose
 * 订阅消息类型: nav_msgs::msg::Odometry
 * 配置参数:
 * - use_simulation (bool): 是否使用仿真里程计，默认true
 * - simulation_odom_topic (string): 仿真里程计话题，默认"/absolute_pose"
 * - real_robot_odom_topic (string): 实机里程计话题，默认"/aft_mapped_to_init"
 * - reset_origin_on_start (bool): 首帧作为PointLIO局部原点，默认true
 * - field_origin_x/y/z/yaw (double): 首帧对应的场地坐标，默认H点(0.75,0.75,0)
 */
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros2_tools/msg/lidar_pose.hpp"

class LidarDataNode : public rclcpp::Node {
public:
    LidarDataNode() : Node("lidar_data_node") {
        // 参数声明
        this->declare_parameter<bool>("use_simulation", false);
        this->declare_parameter<std::string>("simulation_odom_topic", "/absolute_pose");
        this->declare_parameter<std::string>("real_robot_odom_topic", "/aft_mapped_to_init");
        this->declare_parameter<bool>("reset_origin_on_start", true);
        this->declare_parameter<double>("field_origin_x", 0.75);
        this->declare_parameter<double>("field_origin_y", 0.75);
        this->declare_parameter<double>("field_origin_z", 0.0);
        this->declare_parameter<double>("field_origin_yaw", 0.0);
        // 获取参数值
        using_gazebo_ = this->get_parameter("use_simulation").as_bool();
        std::string sim_topic = this->get_parameter("simulation_odom_topic").as_string();
        std::string real_topic = this->get_parameter("real_robot_odom_topic").as_string();
        reset_origin_on_start_ = this->get_parameter("reset_origin_on_start").as_bool();
        field_origin_x_ = this->get_parameter("field_origin_x").as_double();
        field_origin_y_ = this->get_parameter("field_origin_y").as_double();
        field_origin_z_ = this->get_parameter("field_origin_z").as_double();
        field_origin_yaw_ = this->get_parameter("field_origin_yaw").as_double();
        
        // 设置QoS，某些情况下
        //rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 雷达数据发布
        lidar_pub = this->create_publisher<ros2_tools::msg::LidarPose>("lidar_data", 10);
        RCLCPP_INFO(this->get_logger(), "创建发布 lidar_data");

        selected_odom_topic_ = using_gazebo_ ? sim_topic : real_topic;
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            selected_odom_topic_, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ LidarDataNode::odomCallback(msg);});
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() { logOdomWatchdog(); });
        RCLCPP_INFO(
            this->get_logger(),
            "创建%sodom订阅: %s",
            using_gazebo_ ? "仿真" : "实机",
            selected_odom_topic_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "场地坐标映射: reset_origin_on_start=%s, field_origin=(%.2f, %.2f, %.2f), yaw_offset=%.2f rad",
            reset_origin_on_start_ ? "true" : "false",
            field_origin_x_, field_origin_y_, field_origin_z_, field_origin_yaw_);
    }

    // 兼容版odom回调
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        received_odom_ = true;
        msgDispose(msg->pose.pose);
    }

    // 处理并发布LidarPose消息
    void msgDispose(const geometry_msgs::msg::Pose &pose) {
        // 提取位置和姿态
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 归一化到0~2π
        if (roll < 0) roll += 2 * M_PI;
        if (pitch < 0) pitch += 2 * M_PI;
        if (yaw < 0) yaw += 2 * M_PI;

        if (reset_origin_on_start_ && !origin_ready_) {
            origin_x_ = x;
            origin_y_ = y;
            origin_z_ = z;
            origin_yaw_ = yaw;
            origin_ready_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "记录PointLIO首帧为局部原点: raw=(%.2f, %.2f, %.2f), yaw=%.2f rad",
                origin_x_, origin_y_, origin_z_, origin_yaw_);
        }

        const double raw_dx = reset_origin_on_start_ ? x - origin_x_ : x;
        const double raw_dy = reset_origin_on_start_ ? y - origin_y_ : y;
        const double local_z = reset_origin_on_start_ ? z - origin_z_ : z;
        const double yaw_local = reset_origin_on_start_ ? yaw - origin_yaw_ : yaw;
        const double cos_origin = std::cos(origin_yaw_);
        const double sin_origin = std::sin(origin_yaw_);
        const double local_x = reset_origin_on_start_ ? cos_origin * raw_dx + sin_origin * raw_dy : raw_dx;
        const double local_y = reset_origin_on_start_ ? -sin_origin * raw_dx + cos_origin * raw_dy : raw_dy;

        const double cos_offset = std::cos(field_origin_yaw_);
        const double sin_offset = std::sin(field_origin_yaw_);
        const double field_x = field_origin_x_ + cos_offset * local_x - sin_offset * local_y;
        const double field_y = field_origin_y_ + sin_offset * local_x + cos_offset * local_y;
        const double field_z = field_origin_z_ + local_z;
        double field_yaw = yaw_local + field_origin_yaw_;
        field_yaw = std::fmod(field_yaw, 2 * M_PI);
        if (field_yaw < 0) field_yaw += 2 * M_PI;

        // 填充并发布场地坐标系下的LidarPose消息
        lidar_pose.x = field_x;
        lidar_pose.y = field_y;
        lidar_pose.z = field_z;
        lidar_pose.roll = roll;
        lidar_pose.pitch = pitch;
        lidar_pose.yaw = field_yaw;

        lidar_pub->publish(lidar_pose);

        log(field_x, field_y, field_z, roll, pitch, field_yaw); // 低频打印日志
    }

    // 低频打印当前位姿日志
    void log(double x, double y, double z, double roll, double pitch, double yaw) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, // 5秒打印一次
            "Position=(%.2f, %.2f, %.2f), Orientation=(%.2f, %.2f, %.2f) rad",
            x, y, z, roll, pitch, yaw);
    }

    void logOdomWatchdog() {
        if (received_odom_) {
            return;
        }
        RCLCPP_WARN(
            this->get_logger(),
            "等待PointLIO里程计: 未收到 %s，收到后将低频打印 Position/Orientation",
            selected_odom_topic_.c_str());
    }

private:
    bool using_gazebo_; // 仿真开关
    bool received_odom_{false};
    bool reset_origin_on_start_{true};
    bool origin_ready_{false};
    double origin_x_{0.0};
    double origin_y_{0.0};
    double origin_z_{0.0};
    double origin_yaw_{0.0};
    double field_origin_x_{0.75};
    double field_origin_y_{0.75};
    double field_origin_z_{0.0};
    double field_origin_yaw_{0.0};
    std::string selected_odom_topic_;
    
    rclcpp::Publisher<ros2_tools::msg::LidarPose>::SharedPtr lidar_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    ros2_tools::msg::LidarPose lidar_pose;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarDataNode>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar_data_node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
