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
    , home_target_(
        mission_io.get_field_origin_x(),
        mission_io.get_field_origin_y(),
        kHomeAltitude,
        mission_io.get_field_origin_yaw())
{}

// 主循环
void MissionExecutor::run() {
    const int task_id = mission_io_.get_mission_task_id();
    RCLCPP_INFO(
        logger_,
        "[Mission] 任务%d开始: 飞到H(%.3f,%.3f,%.2f) → 等待3s → 执行协同任务 → 返回H → 降落",
        task_id,
        home_target_.get_x(),
        home_target_.get_y(),
        home_target_.get_z());
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
        home_target_.get_x(),
        home_target_.get_y(),
        home_target_.get_z());
    mission_io_.publish_drone_status("TAKEOFF", "fly to H cruise altitude");
    fc_.fly_to_target_pid(home_target_, 20.0f, 0.3f, 20);
    if (mission_io_.get_mission_task_id() == 2) {
        RCLCPP_INFO(logger_, "[TAKEOFF] 任务2起飞完成，跳过3s悬停，切换 FOLLOW_CARRIER");
        current_state_ = State::FOLLOW_CARRIER;
    } else {
        RCLCPP_INFO(logger_, "[TAKEOFF] 到达起始等待点，切换 WAIT_BEFORE_FOLLOW");
        current_state_ = State::WAIT_BEFORE_FOLLOW;
    }
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

// 状态：FOLLOW_CARRIER - 根据地面站任务编号执行全程伴飞或动态起降
void MissionExecutor::on_follow_carrier() {
    const int task_id = mission_io_.get_mission_task_id();

    if (task_id == 1) {
        mission_io_.publish_drone_status("FOLLOW", "task1 chase carrier until stable");
        RCLCPP_INFO(
            logger_,
            "[FOLLOW_CARRIER] 任务1: 巡航伴飞追到小车并稳定 %.1f s",
            kTask1FollowStableSec);
        fc_.follow_carrier_until_stable(
            kTask1FollowTimeoutSec,
            kTask1FollowStableSec,
            kHomeAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec,
            kTask1FollowTolerance);

        mission_io_.publish_drone_status("DropState", "descend to compensated half-height airdrop point");
        RCLCPP_INFO(
            logger_,
            "[DROP_APPROACH] 下降到半空投掷位 %.2fm，补偿 offset=(forward %.3f, left %.3f)，稳定 %.1f s",
            kTask1DropAltitude,
            kAirdropForwardOffset,
            kAirdropLeftOffset,
            kTask1DropStableSec);
        fc_.follow_carrier_until_stable(
            kTask1FollowTimeoutSec,
            kTask1DropStableSec,
            kTask1DropAltitude,
            kAirdropForwardOffset,
            kAirdropLeftOffset,
            kCarrierPoseMaxAgeSec,
            kTask1DropTolerance);

        mission_io_.publish_drone_status("DropState", "airdrop release command published");
        mission_io_.publish_airdrop_command("release");
        RCLCPP_INFO(logger_, "[AIRDROP] 已发布舵机抛投命令: release");

        mission_io_.publish_drone_status("DropState", "holding after airdrop before return");
        RCLCPP_INFO(logger_, "[POST_DROP_HOLD] 投掷完成，原地悬停 %.1f s 后返航", kTask1PostDropHoldSec);
        rclcpp::Rate hold_rate(20);
        const auto hold_start = steady_clock_.now();
        while (rclcpp::ok() && (steady_clock_.now() - hold_start).seconds() < kTask1PostDropHoldSec) {
            const DroneState s = state_.get_state();
            Target hold_target(s.x, s.y, s.z, s.yaw);
            cmd_.publish_position(hold_target);
            hold_rate.sleep();
        }
    } else if (task_id == 2) {
        mission_io_.publish_drone_status("WAIT_CD", "task2 following carrier until C-D segment");
        if (!carrier_.has_recent_carrier_pose(kCarrierPoseMaxAgeSec)
            || !mission_io_.has_recent_car_status(kTask2CarStatusMaxAgeSec)) {
            hold_current_position_until_carrier_ready(
                "WAIT_CARRIER",
                "task2 waiting for /carrier/lidar_pose and /car/status before follow",
                false);
        }
        RCLCPP_INFO(
            logger_,
            "[FOLLOW_CARRIER] 任务2: 起飞后不悬停，以 %.2fm 连续伴飞等待小车进入CD段(wp_index=%d)",
            kTask2WaitCdAltitude,
            kTask2CdWpIndex);
        auto car_reached_cd = [this]() {
            if (mission_io_.has_recent_car_status(kTask2CarStatusMaxAgeSec)) {
                const int car_wp = mission_io_.get_car_wp_index();
                const int car_task = mission_io_.get_car_task_id();
                if (car_task == 2 && car_wp >= kTask2CdWpIndex) {
                    RCLCPP_INFO(logger_, "[FOLLOW_CARRIER] 小车已进入CD段: task=%d wp_index=%d", car_task, car_wp);
                    return true;
                }
            } else {
                RCLCPP_WARN_THROTTLE(
                    logger_, steady_clock_, 1000,
                    "[FOLLOW_CARRIER] 等待 /car/status 以判断CD段");
            }

            return false;
        };
        fc_.follow_carrier_until(
            kTask2WaitCdTimeoutSec,
            car_reached_cd,
            kTask2WaitCdAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec,
            20,
            false);
        if (!mission_io_.has_recent_car_status(kTask2CarStatusMaxAgeSec)
            || mission_io_.get_car_task_id() != 2
            || mission_io_.get_car_wp_index() < kTask2CdWpIndex) {
            RCLCPP_WARN(
                logger_,
                "[FOLLOW_CARRIER] 未确认小车进入CD段，进入原地等待保护，不自动返航/降落");
            hold_current_position_until_carrier_ready(
                "WAIT_CARRIER",
                "task2 did not reach C-D segment before timeout; holding for recovery",
                true);
        }

        mission_io_.publish_drone_status("LAND_ON_CARRIER", "dynamic landing on carrier");
        RCLCPP_INFO(logger_, "[LAND_ON_CARRIER] 任务2: CD段伴飞缓降到航母平台上方");
        fc_.follow_carrier(
            kTask2CarrierLandSec,
            kCarrierLandingAltitude,
            kCarrierForwardOffset,
            kCarrierLeftOffset,
            kCarrierPoseMaxAgeSec);

        mission_io_.publish_drone_status("STAY_ON_CARRIER", "holding on carrier for 6 seconds");
        RCLCPP_INFO(logger_, "[STAY_ON_CARRIER] 发布小车投影位置 z=%.1f 模拟不可达降落，持续 %.1f s",
            kCarrierLandingProjectionZ, kTask2StayOnCarrierSec);
        rclcpp::Rate hold_rate(20);
        const auto stay_start = steady_clock_.now();
        while (rclcpp::ok() && (steady_clock_.now() - stay_start).seconds() < kTask2StayOnCarrierSec) {
            const DroneState carrier_pose = carrier_.get_carrier_pose();
            Target carrier_projection(
                carrier_pose.x,
                carrier_pose.y,
                kCarrierLandingProjectionZ,
                carrier_pose.yaw);
            cmd_.publish_position(carrier_projection);
            hold_rate.sleep();
        }

        mission_io_.publish_drone_status("TAKEOFF_FROM_CARRIER", "takeoff from carrier to cruise altitude");
        RCLCPP_INFO(logger_, "[TAKEOFF_FROM_CARRIER] 从航母平台复飞到 %.2fm 巡航高度", kHomeAltitude);
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

void MissionExecutor::hold_current_position_until_carrier_ready(
    const char* phase,
    const char* detail,
    bool require_cd_segment) {
    rclcpp::Rate hold_rate(20);
    const DroneState s = state_.get_state();
    Target hold_target(s.x, s.y, s.z, s.yaw);
    mission_io_.publish_drone_status(phase, detail);
    RCLCPP_WARN(
        logger_,
        "[%s] %s，保持当前位置: (%.2f, %.2f, %.2f)",
        phase,
        detail,
        hold_target.get_x(),
        hold_target.get_y(),
        hold_target.get_z());

    while (rclcpp::ok()) {
        const bool inputs_ready =
            carrier_.has_recent_carrier_pose(kCarrierPoseMaxAgeSec)
            && mission_io_.has_recent_car_status(kTask2CarStatusMaxAgeSec);
        const bool cd_ready =
            inputs_ready
            && mission_io_.get_car_task_id() == 2
            && mission_io_.get_car_wp_index() >= kTask2CdWpIndex;
        if (inputs_ready && (!require_cd_segment || cd_ready)) {
            mission_io_.publish_drone_status("WAIT_CD", "carrier/status recovered, resume task2 follow");
            RCLCPP_INFO(logger_, "[%s] 小车数据%s，继续任务2",
                phase,
                require_cd_segment ? "已到CD段" : "恢复");
            return;
        }

        cmd_.publish_position(hold_target);
        RCLCPP_WARN_THROTTLE(
            logger_, steady_clock_, 2000,
            "[%s] 等待 /carrier/lidar_pose 和 /car/status%s，持续原地悬停",
            phase,
            require_cd_segment ? " 进入CD段" : "");
        hold_rate.sleep();
    }
}

// 状态：RETURN_HOME - 回到起始等待点 (0, 0, 1)
void MissionExecutor::on_return_home() {
    RCLCPP_INFO(
        logger_,
        "[RETURN_HOME] 返回H起降点上方: (%.2f, %.2f, %.2f)",
        home_target_.get_x(),
        home_target_.get_y(),
        home_target_.get_z());
    mission_io_.publish_drone_status("RETURN_HOME", "return to H before landing");
    fc_.fly_to_target_pid(home_target_, 30.0f, 0.3f, 20);
    RCLCPP_INFO(logger_, "[RETURN_HOME] 已回到起始等待点，切换 LAND");
    current_state_ = State::LAND;
}

// 状态：LAND
void MissionExecutor::on_land() {
    Target landing_approach(
        home_target_.get_x(),
        home_target_.get_y(),
        kLandingApproachAltitude,
        home_target_.get_yaw());

    RCLCPP_INFO(
        logger_,
        "[LAND] 先到H近地点: (%.2f, %.2f, %.2f), 判定半径 %.2fm",
        landing_approach.get_x(),
        landing_approach.get_y(),
        landing_approach.get_z(),
        kLandingApproachTolerance);
    mission_io_.publish_drone_status("LAND_APPROACH", "precise approach above H before descent");
    fc_.fly_to_target_pid(
        landing_approach,
        30.0f,
        0.5f,
        20,
        kLandingApproachTolerance);

    RCLCPP_INFO(logger_, "[LAND] 近地点对位完成，开始稳定下降，速度 %.2f m/s，持续 %.1f s",
        kLandVz, kLandDuration);
    mission_io_.publish_drone_status("LAND", "stable vertical descent at H");
    Velocity land_vel(0.0f, 0.0f, kLandVz);
    fc_.fly_by_vel_duration(land_vel, kLandDuration);
    RCLCPP_INFO(logger_, "[LAND] 降落完成");
    current_state_ = State::DONE;
}
