#pragma once

#include <mutex>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros2_tools/msg/lidar_pose.hpp"
#include "messages/msg/platform_target.hpp"

#include "layer1_hal/i_state_provider.hpp"
#include "layer1_hal/i_command_publisher.hpp"
#include "layer1_hal/i_dvs_avoid_provider.hpp"
#include "layer1_hal/i_carrier_pose_provider.hpp"
#include "layer1_hal/i_mission_io.hpp"

/**
 * @brief 硬件抽象层（Layer 1 · HAL Concrete）
 *
 * 整个系统直接操作 ROS2 通信原语（pub/sub/client）的中间层。
 * 实现三个接口：
 *   - IStateProvider    — 位姿状态
 *   - ICommandPublisher — 收发指令
 *   - ICarrierPoseProvider — 空地协同目标位姿
 * 
 *   - 对外暴露的 MAVRos服务 仅供 DroneSystem 的预飞行阶段。
 */
class DroneHAL
    : public rclcpp::Node
    , public IStateProvider
    , public ICommandPublisher
    , public IDvsAvoidProvider
    , public ICarrierPoseProvider
    , public IMissionIO
{
public:
    explicit DroneHAL();

    // 状态提供接口 IStateProvider
    [[nodiscard]] DroneState get_state() const override;
    [[nodiscard]] bool       has_state() const override;

    // 指令发布接口 ICommandPublisher
    void publish_position(Target& target)   override;
    void publish_velocity(Velocity& velocity) override;
    [[nodiscard]] bool uses_planar_position_control() const override;

    // DVS 规避提供接口 IDvsAvoidProvider
    [[nodiscard]] geometry_msgs::msg::Twist get_dvs_avoid_cmd() const override;
    [[nodiscard]] bool has_recent_dvs_avoid(double max_age_sec) const override;
    [[nodiscard]] int64_t get_last_dvs_detect_time_ns() const override;

    // 空地协同目标位姿提供接口 ICarrierPoseProvider
    [[nodiscard]] DroneState get_carrier_pose() const override;
    [[nodiscard]] bool has_recent_carrier_pose(double max_age_sec) const override;

    // 地面站任务命令/状态接口 IMissionIO
    [[nodiscard]] bool has_mission_start() const override;
    [[nodiscard]] int get_mission_task_id() const override;
    [[nodiscard]] float get_field_origin_x() const override { return field_origin_x_; }
    [[nodiscard]] float get_field_origin_y() const override { return field_origin_y_; }
    [[nodiscard]] float get_field_origin_z() const override { return field_origin_z_; }
    [[nodiscard]] float get_field_origin_yaw() const override { return field_origin_yaw_; }
    [[nodiscard]] int get_car_wp_index() const override;
    [[nodiscard]] int get_car_task_id() const override;
    [[nodiscard]] bool has_recent_car_status(double max_age_sec) const override;
    void clear_mission_start() override;
    void publish_drone_status(const std::string& phase, const std::string& detail) override;
    void publish_airdrop_command(const std::string& action) override;

    // MAVROS 服务接口（供 DroneSystem 触发）
    bool request_arm(bool arm = true); // 请求px4解锁并等待响应，返回飞控是否接受
    bool request_set_mode(const std::string& mode); // 请求切换px4模式并等待响应，返回飞控是否接受

    // 查询 MAVROS 状态
    [[nodiscard]] mavros_msgs::msg::State get_mavros_state() const;
    [[nodiscard]] bool requires_mavros_preflight() const;
    [[nodiscard]] std::string get_platform_mode_name() const;

private:
    enum class PlatformMode {
        Px4MavrosDrone,
        Px4MavrosDiffCar,
        CustomAckermannCar
    };

    // ===== 回调组 =====
    void lidar_cb(const ros2_tools::msg::LidarPose::SharedPtr msg);
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg);
    void dvs_detection_cb(const std_msgs::msg::String::SharedPtr msg);
    void dvs_avoid_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void carrier_pose_cb(const ros2_tools::msg::LidarPose::SharedPtr msg);
    void car_status_cb(const std_msgs::msg::String::SharedPtr msg);
    void mission_command_cb(const std_msgs::msg::String::SharedPtr msg);

    static bool extract_json_int64(const std::string& json, const std::string& key, int64_t& out);
    static bool extract_json_bool(const std::string& json, const std::string& key, bool& out);
    static bool extract_json_int(const std::string& json, const std::string& key, int& out);
    static bool extract_json_string(const std::string& json, const std::string& key, std::string& out);
    static PlatformMode parse_platform_mode(const std::string& mode_name);
    static float normalize_angle(float angle_rad);
    void log_dvs_pipeline_latency_if_applicable(const char* command_type);
    Target field_target_to_px4_local(const Target& target) const;
    Velocity field_velocity_to_px4_local(const Velocity& velocity) const;
    void publish_px4_drone_position(Target& target);
    void publish_px4_drone_velocity(Velocity& velocity);
    void publish_car_position_target(const Target& target, bool use_custom_ackermann);
    void publish_car_velocity_target(Velocity& velocity, bool use_custom_ackermann);
    messages::msg::PlatformTarget make_platform_target_from_position(const Target& target) const;
    messages::msg::PlatformTarget make_platform_target_from_velocity(const Velocity& velocity) const;
    void publish_status_message(const std::string& phase, const std::string& detail);
    void publish_status_heartbeat();

    // ===== 发布器组 =====
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<messages::msg::PlatformTarget>::SharedPtr    platform_target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            drone_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            airdrop_cmd_pub_;

    // ===== 订阅器 =====
    rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr    lidar_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr       state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         dvs_detection_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr     dvs_avoid_sub_;
    rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr    carrier_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         car_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         mission_command_sub_;
    rclcpp::TimerBase::SharedPtr                                   status_timer_;

    // ===== 服务客户端 =====
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr     set_mode_client_;

    // 结果状态
    mutable std::mutex state_mutex_;
    DroneState         state_{};
    bool               has_state_{false};

    // DVS 规避结果
    mutable std::mutex          dvs_mutex_;
    geometry_msgs::msg::Twist   dvs_avoid_cmd_{};
    rclcpp::Time                dvs_avoid_rx_time_{0, 0, RCL_SYSTEM_TIME};
    bool                        has_dvs_avoid_{false};
    int64_t                     last_dvs_detect_time_ns_{0};
    int64_t                     last_latency_logged_detect_ns_{0};

    // 空地协同小车/航母平台位姿
    mutable std::mutex carrier_pose_mutex_;
    DroneState         carrier_pose_{};
    rclcpp::Time       carrier_pose_rx_time_{0, 0, RCL_SYSTEM_TIME};
    bool               has_carrier_pose_{false};

    // 地面站任务命令
    mutable std::mutex mission_mutex_;
    bool               mission_start_requested_{false};
    int                mission_task_id_{1};

    mutable std::mutex car_status_mutex_;
    int                car_wp_index_{-1};
    int                car_task_id_{0};
    rclcpp::Time       car_status_rx_time_{0, 0, RCL_SYSTEM_TIME};
    bool               has_car_status_{false};

    mutable std::mutex status_mutex_;
    std::string        last_status_phase_{"BOOT"};
    std::string        last_status_detail_{"drone node alive"};

    // MAVRos状态
    mutable std::mutex      mavros_mutex_;
    mavros_msgs::msg::State mavros_state_{};

    PlatformMode platform_mode_{PlatformMode::Px4MavrosDrone};
    std::string platform_mode_name_{"px4_drone"};
    std::string position_setpoint_topic_{"/mavros/setpoint_position/local"};
    std::string velocity_setpoint_topic_{"/mavros/setpoint_velocity/cmd_vel"};
    std::string platform_target_topic_{"/platform/target"};
    std::string lidar_pose_topic_{"/drone/lidar_data"};
    std::string carrier_pose_topic_{"/carrier/lidar_pose"};
    std::string car_status_topic_{"/car/status"};
    std::string mission_command_topic_{"/mission/command"};
    std::string drone_status_topic_{"/drone/status"};
    std::string airdrop_cmd_topic_{"/drone/airdrop_cmd"};
    float car_position_kp_speed_{0.8f};
    float car_yaw_kp_{1.5f};
    float car_max_speed_mps_{0.6f};
    float car_max_yaw_rate_radps_{1.0f};
    float car_xy_tolerance_m_{0.08f};
    bool field_frame_enabled_{false};
    float field_origin_x_{1.125f};
    float field_origin_y_{1.125f};
    float field_origin_z_{0.0f};
    float field_origin_yaw_{1.5707963f};
};
