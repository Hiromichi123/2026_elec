#pragma once

#include <string>

/**
 * @brief 地面站任务命令与飞机状态/抛投输出接口。
 */
class IMissionIO {
public:
    virtual ~IMissionIO() = default;

    [[nodiscard]] virtual bool has_mission_start() const = 0;
    [[nodiscard]] virtual int get_mission_task_id() const = 0;
    virtual void clear_mission_start() = 0;

    virtual void publish_drone_status(const std::string& phase,
                                      const std::string& detail) = 0;
    virtual void publish_airdrop_command(const std::string& action) = 0;
};
