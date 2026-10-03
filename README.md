# 2026年江苏省电子设计竞赛飞控——陆地航母

本仓库将三个项目整合为统一工程：

- `quadcopter/`：原 `Hiromichi123/2026_elec`（无人机飞控与任务执行）
- `car/`：来自 `Hiromichi123/car_2026`（地面运载车）
- `land/`：来自 `Hiromichi123/2026_land`（地面基站）

> 说明：为避免覆盖冲突，三个项目均保持在独立子目录中，`ros2_tools`、`.gitignore`、README 等同名内容均按子项目隔离保留。

## 总体架构

组合系统由三端协同：

1. 无人机端执行飞行任务与协同逻辑。
2. 地面运载车端执行轨迹导航并发布车体位姿。
3. 地面基站端提供任务触发与状态显示。

在 `quadcopter/COOPERATIVE_MISSION.md` 中记录了三端 ROS2 协同话题（如 `/mission/command`、`/drone/status`、`/car/status`、`/carrier/lidar_pose`）与任务流程说明。

## 目录结构

```text
.
├── README.md                  # 本整合说明
├── quadcopter/                # 原 2026_elec 全部内容
│   ├── README.md
│   ├── COOPERATIVE_MISSION.md
│   ├── core_2026/
│   ├── messages/
│   └── ros2_tools/
├── car/                       # 原 car_2026 全部内容
│   ├── README.md
│   ├── line_follower/
│   ├── robot_real/
│   └── ros2_tools/
└── land/                      # 原 2026_land 全部内容
    ├── land_station/
    └── ros2_tools/
```

## 无人机（quadcopter）

- 来源：原 `2026_elec` 仓库完整迁移。
- 主要包：`core_2026`、`messages`、`ros2_tools`。
- 参考文档：
  - `quadcopter/README.md`
  - `quadcopter/COOPERATIVE_MISSION.md`

可确认的入口（来自现有文档）：

```bash
cd quadcopter
colcon build --packages-select core_2026 --allow-overriding messages ros2_tools --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 launch core_2026 core_launch.py carrier_pose_topic:=/carrier/lidar_pose
```

## 地面运载车（car）

- 来源：`car_2026` 原仓库内容完整保留。
- 参考文档：`car/README.md`
- 主要包：`line_follower`、`robot_real`、`ros2_tools`

可确认的入口（来自 `car/README.md`）：

```bash
cd car
source ~/AIC_car_2025/install/setup.bash
ros2 run ros2_tools hardware_bridge_node
ros2 launch line_follower navigate.launch.py
```

`car/README.md` 中还保留了 `/cmd_vel` 控制示例（前进、后退、转向、停止等）。

## 地面基站（land）

- 来源：`2026_land` 原仓库内容完整保留。
- 主要包：`land_station`、`ros2_tools`
- 启动文件：`land/land_station/launch/ground_station.launch.py`

可确认的入口（来自代码与协同文档）：

```bash
cd land
colcon build --packages-select ros2_tools land_station --allow-overriding ros2_tools
source install/setup.bash
ros2 launch land_station ground_station.launch.py
```

如缺少 UI 依赖，协同文档给出的补充命令为：

```bash
python3 -m pip install pygame
```

## 三者协同关系

按照 `quadcopter/COOPERATIVE_MISSION.md`：

- 地面基站发布任务命令（`/mission/command`）。
- 无人机与小车分别上报状态（`/drone/status`、`/car/status`）。
- 小车发布位姿（`/carrier/lidar_pose`）供无人机协同控制。
- 无人机在任务流程中执行跟随、返航、降落/抛投等动作。

## 开发环境与依赖

基于现有仓库文件与文档可确认：

- ROS2（文档示例使用 Humble 环境）
- colcon
- Python（`land_station` 为 Python 包，UI 可能需要 `pygame`）
- CMake（`core_2026`、`ros2_tools` 等 C++/ROS2 包）

## 后续完善项

1. 在统一工作区下补充三端一键构建/启动脚本，减少手动切换目录。
2. 对三端联合联调流程（网络、时序、异常恢复）补充可复现操作手册。
3. 对 `COOPERATIVE_MISSION.md` 中的任务边界项（如抛投执行节点、动态落车参数）继续标定并固化。
4. 若后续新增 `land` 独立 README，可在根 README 增加直接引用。
