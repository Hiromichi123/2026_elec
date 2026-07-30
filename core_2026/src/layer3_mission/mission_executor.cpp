#include "layer3_mission/mission_executor.hpp"

using namespace fly_to_target_args;

MissionExecutor::MissionExecutor(
    FlightController& fc,
    IStateProvider&   state,
    IDvsAvoidProvider& dvs,
    ICarrierPoseProvider& carrier,
    ICommandPublisher& cmd,
    rclcpp::Logger    logger,
    float             default_altitude)
    : fc_(fc)
    , state_(state)
    , dvs_(dvs)
    , carrier_(carrier)
    , cmd_(cmd)
    , logger_(logger)
    , default_altitude_(default_altitude)
    , takeoff_target_(0.0f, 0.0f, kHoverAltitude, 0.0f)
    , hover_target_(0.0f, 0.0f, kHoverAltitude, 0.0f)
{}

// 主循环
void MissionExecutor::run() {
    RCLCPP_INFO(logger_, "[Mission] 任务开始: 原地起飞 → 空地协同跟随航母平台50s → 降落");
    while (rclcpp::ok() && current_state_ != State::DONE) {
        switch (current_state_) {
            case State::TAKEOFF:        on_takeoff();        break;
            case State::FOLLOW_CARRIER: on_follow_carrier(); break;
            case State::LAND:           on_land();           break;
            case State::DONE:           break;
            default:                    break;
        }
    }
    RCLCPP_INFO(logger_, "[Mission] 任务完成");
}

// 状态：TAKEOFF - 原地起飞到 1.70m
void MissionExecutor::on_takeoff() {
    const auto s = state_.get_state();
    hover_anchor_x_ = s.x;
    hover_anchor_y_ = s.y;
    hover_anchor_yaw_ = s.yaw;
    takeoff_target_ = Target(hover_anchor_x_, hover_anchor_y_, kHoverAltitude, hover_anchor_yaw_);

    RCLCPP_INFO(logger_, "[TAKEOFF] 原地上升至 %.2f m", kHoverAltitude);
    fc_.fly_to_target(target = takeoff_target_);
    hover_target_ = Target(hover_anchor_x_, hover_anchor_y_, kHoverAltitude, hover_anchor_yaw_);
    RCLCPP_INFO(logger_, "[TAKEOFF] 到达 %.2f m，切换 FOLLOW_CARRIER", kHoverAltitude);
    current_state_ = State::FOLLOW_CARRIER;
}

// 状态：FOLLOW_CARRIER - 跟随同 ROS_DOMAIN_ID 下发布的小车/航母位姿
void MissionExecutor::on_follow_carrier() {
    RCLCPP_INFO(
        logger_,
        "[FOLLOW_CARRIER] 开始跟随航母平台: altitude=%.2f duration=%.1f max_age=%.2f",
        kHoverAltitude,
        kCarrierFollowDurationSec,
        kCarrierPoseMaxAgeSec);

    fc_.follow_carrier(
        kCarrierFollowDurationSec,
        kHoverAltitude,
        kCarrierForwardOffset,
        kCarrierLeftOffset,
        kCarrierPoseMaxAgeSec);

    RCLCPP_INFO(logger_, "[FOLLOW_CARRIER] 跟随结束，切换 LAND");
    current_state_ = State::LAND;
}

// 状态：LAND
void MissionExecutor::on_land() {
    RCLCPP_INFO(logger_, "[LAND] 开始降落，速度 %.2f m/s，持续 %.1f s",
        kLandVz, kLandDuration);
    Velocity land_vel(0.0f, 0.0f, kLandVz);
    fc_.fly_by_vel_duration(land_vel, kLandDuration);
    RCLCPP_INFO(logger_, "[LAND] 降落完成");
    current_state_ = State::DONE;
}
