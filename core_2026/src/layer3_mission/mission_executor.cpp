#include "layer3_mission/mission_executor.hpp"

using namespace fly_to_target_args;

MissionExecutor::MissionExecutor(
    FlightController& fc,
    IStateProvider&   state,
    IDvsAvoidProvider& dvs,
    ICarrierPoseProvider& carrier,
    IMissionIO&       mission_io,
    ICommandPublisher& cmd,
    rclcpp::Logger    logger,
    float             default_altitude)
    : fc_(fc)
    , state_(state)
    , dvs_(dvs)
    , carrier_(carrier)
    , mission_io_(mission_io)
    , cmd_(cmd)
    , logger_(logger)
    , default_altitude_(default_altitude)
    , home_target_(kHomeX, kHomeY, kHomeAltitude, kHomeYaw)
{}

// 主循环
void MissionExecutor::run() {
    const int task_id = mission_io_.get_mission_task_id();
    RCLCPP_INFO(
        logger_,
        "[Mission] 任务%d开始: 飞到H(0.75,0.75,1.50) → 等待3s → 执行协同任务 → 返回H → 降落",
        task_id);
    mission_io_.publish_drone_status("MISSION_START", "mission state machine started");
    while (rclcpp::ok() && current_state_ != State::DONE) {
        switch (current_state_) {
            case State::TAKEOFF:            on_takeoff();            break;
            case State::WAIT_BEFORE_FOLLOW: on_wait_before_follow(); break;
            case State::FOLLOW_CARRIER:     on_follow_carrier();     break;
            case State::RETURN_HOME:        on_return_home();        break;
            case State::LAND:               on_land();               break;
            case State::DONE:               break;
            default:                        break;
        }
    }
    mission_io_.publish_drone_status("DONE", "mission complete");
    RCLCPP_INFO(logger_, "[Mission] 任务完成");
}

// 状态：TAKEOFF - 飞到起始等待点 (0, 0, 1)
void MissionExecutor::on_takeoff() {
    RCLCPP_INFO(
        logger_,
        "[TAKEOFF] 飞到H起降点上方: (%.2f, %.2f, %.2f)",
        kHomeX,
        kHomeY,
        kHomeAltitude);
    mission_io_.publish_drone_status("TAKEOFF", "fly to H cruise altitude");
    fc_.fly_to_target(target = home_target_);
    RCLCPP_INFO(logger_, "[TAKEOFF] 到达起始等待点，切换 WAIT_BEFORE_FOLLOW");
    current_state_ = State::WAIT_BEFORE_FOLLOW;
}

// 状态：WAIT_BEFORE_FOLLOW - 起飞后持续悬停等待 3s
void MissionExecutor::on_wait_before_follow() {
    rclcpp::Rate rate(20);
    const auto start_time = steady_clock_.now();
    mission_io_.publish_drone_status("HOVER", "stable hover before carrier follow");

    while (rclcpp::ok()) {
        const double elapsed = (steady_clock_.now() - start_time).seconds();
        if (elapsed >= kPreFollowWaitSec) {
            break;
        }

        cmd_.publish_position(home_target_);
        RCLCPP_INFO_THROTTLE(
            logger_, steady_clock_, 1000,
            "[WAIT_BEFORE_FOLLOW] 悬停等待，剩余 %.1f s",
            kPreFollowWaitSec - static_cast<float>(elapsed));
        rate.sleep();
    }

    RCLCPP_INFO(logger_, "[WAIT_BEFORE_FOLLOW] 等待完成，切换 FOLLOW_CARRIER");
    current_state_ = State::FOLLOW_CARRIER;
}

// 状态：FOLLOW_CARRIER - 根据地面站任务编号执行伴飞抛投或动态起降
void MissionExecutor::on_follow_carrier() {
    const int task_id = mission_io_.get_mission_task_id();

    if (task_id == 1) {
        mission_io_.publish_drone_status("FOLLOW", "task1 carrier formation before airdrop");
        RCLCPP_INFO(logger_, "[FOLLOW_CARRIER] 任务1: 建立伴飞并准备抛投");
        fc_.follow_carrier(
            kTask1PreDropFollowSec,
            kHomeAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec);

        mission_io_.publish_drone_status("AIRDROP", "airdrop command published");
        mission_io_.publish_airdrop_command("release");
        RCLCPP_INFO(logger_, "[AIRDROP] 已发布抛投命令");

        fc_.follow_carrier(
            kTask1PostDropFollowSec,
            kHomeAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec);
    } else if (task_id == 2) {
        mission_io_.publish_drone_status("FOLLOW", "task2 searching and following carrier");
        RCLCPP_INFO(logger_, "[FOLLOW_CARRIER] 任务2: 巡航寻找并跟随小车");
        fc_.follow_carrier(
            kTask2SearchFollowSec,
            kHomeAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec);

        mission_io_.publish_drone_status("LAND_ON_CARRIER", "dynamic landing on carrier");
        RCLCPP_INFO(logger_, "[LAND_ON_CARRIER] 任务2: 动态降落到小车平台");
        fc_.follow_carrier(
            kTask2CarrierLandSec,
            kCarrierLandingAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec);

        mission_io_.publish_drone_status("STAY_ON_CARRIER", "holding on carrier for 5 seconds");
        rclcpp::Rate hold_rate(20);
        const auto stay_start = steady_clock_.now();
        while (rclcpp::ok() && (steady_clock_.now() - stay_start).seconds() < kTask2StayOnCarrierSec) {
            Velocity stop(0.0f, 0.0f, 0.0f, 0.0f);
            cmd_.publish_velocity(stop);
            hold_rate.sleep();
        }

        mission_io_.publish_drone_status("TAKEOFF_FROM_CARRIER", "takeoff from carrier to cruise altitude");
        fc_.follow_carrier(
            kTask2CarrierTakeoffSec,
            kHomeAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec);
    } else {
        mission_io_.publish_drone_status("ERROR", "unsupported mission task id");
        RCLCPP_WARN(logger_, "[FOLLOW_CARRIER] 不支持的任务编号: %d", task_id);
    }

    RCLCPP_INFO(
        logger_,
        "[FOLLOW_CARRIER] 任务%d协同阶段结束，切换 RETURN_HOME",
        task_id);
    current_state_ = State::RETURN_HOME;
}

// 状态：RETURN_HOME - 回到起始等待点 (0, 0, 1)
void MissionExecutor::on_return_home() {
    RCLCPP_INFO(
        logger_,
        "[RETURN_HOME] 返回H起降点上方: (%.2f, %.2f, %.2f)",
        kHomeX,
        kHomeY,
        kHomeAltitude);
    mission_io_.publish_drone_status("RETURN_HOME", "return to H before landing");
    fc_.fly_to_target(target = home_target_);
    RCLCPP_INFO(logger_, "[RETURN_HOME] 已回到起始等待点，切换 LAND");
    current_state_ = State::LAND;
}

// 状态：LAND
void MissionExecutor::on_land() {
    RCLCPP_INFO(logger_, "[LAND] 开始降落，速度 %.2f m/s，持续 %.1f s",
        kLandVz, kLandDuration);
    mission_io_.publish_drone_status("LAND", "landing at H");
    Velocity land_vel(0.0f, 0.0f, kLandVz);
    fc_.fly_by_vel_duration(land_vel, kLandDuration);
    RCLCPP_INFO(logger_, "[LAND] 降落完成");
    current_state_ = State::DONE;
}
