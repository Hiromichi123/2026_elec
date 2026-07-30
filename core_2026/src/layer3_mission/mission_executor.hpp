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
	    static constexpr float  kHomeX                    = 0.75f;  // H点 x: 75cm
	    static constexpr float  kHomeY                    = 0.75f;  // H点 y: 75cm
	    static constexpr float  kHomeAltitude             = 1.50f;  // 巡航高度: 150cm
	    static constexpr float  kHomeYaw                  = 0.0f;   // 起始等待点偏航
	    static constexpr float  kPreFollowWaitSec         = 3.0f;   // 跟随前等待时间
	    static constexpr float  kTask1PreDropFollowSec    = 2.0f;   // 任务一抛投前稳定伴飞
	    static constexpr float  kTask1PostDropFollowSec   = 2.0f;   // 任务一抛投后短暂伴飞
	    static constexpr float  kTask2SearchFollowSec     = 5.0f;   // 任务二巡航寻找/跟随
	    static constexpr float  kTask2CarrierLandSec      = 7.0f;   // 任务二动态降落到车上
	    static constexpr float  kTask2StayOnCarrierSec    = 5.0f;   // 任务二车上停留
	    static constexpr float  kTask2CarrierTakeoffSec   = 3.0f;   // 任务二从车上复飞
	    static constexpr float  kCarrierPoseMaxAgeSec     = 0.5f;   // 小车位姿最大允许延迟
	    static constexpr float  kCarrierForwardOffset     = 0.0f;   // 沿小车前方偏移
	    static constexpr float  kCarrierLeftOffset        = 0.0f;   // 沿小车左侧偏移
	    static constexpr float  kCarrierLandingAltitude   = 0.15f;  // 车载平台上方低高度
    static constexpr float  kLandVz           = -0.20f;   // 降落速度
    static constexpr float  kLandDuration     = 5.0f;     // 降落持续秒

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
