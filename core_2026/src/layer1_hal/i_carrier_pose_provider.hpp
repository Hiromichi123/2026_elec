#pragma once

#include "layer0_common/drone_state.hpp"

/**
 * @brief 空地协同目标位姿读取接口。
 *
 * 当前用于接收地面小车/航母平台的位姿，消息格式复用 ros2_tools::msg::LidarPose。
 */
class ICarrierPoseProvider {
public:
    virtual ~ICarrierPoseProvider() = default;

    // 获取最近一次收到的小车/航母平台位姿快照。
    [[nodiscard]] virtual DroneState get_carrier_pose() const = 0;

    // 是否在 max_age_sec 内收到过小车/航母平台位姿。
    [[nodiscard]] virtual bool has_recent_carrier_pose(double max_age_sec) const = 0;
};
