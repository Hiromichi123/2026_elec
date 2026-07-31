#pragma once

#include <rclcpp/rclcpp.hpp>

#include "layer2_control/flight_controller.hpp"
#include "layer1_hal/i_state_provider.hpp"
#include "layer1_hal/i_dvs_avoid_provider.hpp"
#include "layer1_hal/i_command_publisher.hpp"
#include "layer1_hal/i_carrier_pose_provider.hpp"
#include "layer1_hal/i_mission_io.hpp"
#include "layer0_common/target.hpp"
#include "layer0_common/velocity.hpp"

/**
 * @brief 任务执行层
 *
 * - 通过接口获取状态和视觉数据，不依赖任何具体硬件类。
 */
class MissionExecutor {
public:
    MissionExecutor(FlightController&  fc,     // 飞行控制器
	                    IStateProvider&    state,  // 状态接口
	                    IDvsAvoidProvider& dvs,    // DVS规避接口
	                    ICarrierPoseProvider& carrier, // 空地协同小车/航母位姿接口
	                    IMissionIO&        mission_io, // 地面站任务/状态接口
	                    ICommandPublisher& cmd,    // 指令发布接口（悬停直发setpoint）
                    rclcpp::Logger     logger, // DroneHAL日志记录器
                    float              default_altitude = 1.2f);

    void run(); // 开始执行任务

private:
    // ===== 状态机组 =====
    enum class State {
        TAKEOFF,            // 飞到起始等待点
        WAIT_BEFORE_FOLLOW, // 起飞后悬停等待
        FOLLOW_CARRIER,     // 跟随空地协同小车/航母平台
        RETURN_HOME,        // 返回起始等待点
        LAND,               // 缓慢降落
        DONE                // 任务完成
    };

    // ===== 状态方法组 =====
    void on_takeoff();
    void on_wait_before_follow();
    void on_follow_carrier();
    void on_return_home();
    void on_land();

    // ===== 具名常量组 =====
	    static constexpr float  kHomeAltitude             = 1.40f;  // 巡航高度: 140cm
	    static constexpr float  kPreFollowWaitSec         = 3.0f;   // 跟随前等待时间
	    static constexpr float  kTask1FollowTimeoutSec    = 60.0f;  // 任务一：最长追踪时间
	    static constexpr float  kTask1FollowStableSec     = 1.0f;   // 任务一：追到后稳定伴飞时间
	    static constexpr float  kTask1FollowTolerance     = 0.25f;  // 任务一：追到判定半径
	    static constexpr float  kTask2SearchFollowSec     = 5.0f;   // 任务二巡航寻找/跟随
	    static constexpr float  kTask2WaitCdFollowSliceSec = 0.5f;  // 任务二等待CD段时的伴飞片段
	    static constexpr int    kTask2CdWpIndex           = 8;      // 小车C->D直线段航点索引
	    static constexpr float  kTask2CarStatusMaxAgeSec  = 1.0f;   // 小车状态最大允许延迟
	    static constexpr float  kTask2CarrierLandSec      = 7.0f;   // 任务二CD段动态缓降
	    static constexpr float  kTask2StayOnCarrierSec    = 5.0f;   // 任务二车上停留
	    static constexpr float  kTask2CarrierTakeoffSec   = 3.0f;   // 任务二从车上复飞
	    static constexpr float  kCarrierPoseMaxAgeSec     = 0.5f;   // 小车位姿最大允许延迟
	    static constexpr float  kCarrierForwardOffset     = 0.0f;   // 沿小车前方偏移
	    static constexpr float  kCarrierLeftOffset        = 0.0f;   // 沿小车左侧偏移
	    static constexpr float  kCarrierLandingAltitude   = 0.15f;  // 车载平台上方低高度
	    static constexpr float  kCarrierLandingProjectionZ = -5.0f; // 模拟降落不可达投影高度
    static constexpr float  kLandVz           = -0.20f;   // 降落速度
    static constexpr float  kLandDuration     = 5.0f;     // 降落持续秒
    static constexpr float  kLandingApproachAltitude = 0.25f; // 降落前H点近地点高度
    static constexpr float  kLandingApproachTolerance = 0.08f; // 近地点精确判定半径

    // ===== 成员组 =====
    FlightController& fc_;
	    IStateProvider&   state_;
	    IDvsAvoidProvider& dvs_;
	    ICarrierPoseProvider& carrier_;
	    IMissionIO&       mission_io_;
	    ICommandPublisher& cmd_;
    rclcpp::Logger    logger_;

    float default_altitude_;
    State current_state_{State::TAKEOFF};

    Target home_target_;  // 起始等待点 / 返航点

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
};
