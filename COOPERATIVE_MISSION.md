# 2026陆空协同任务开发说明

## 目标

本次开发把三个独立仓库对齐为同一套局域网 ROS2 协同协议：

- `2026_elec`: 飞机上位机任务编排与 PX4/MAVROS 控制。
- `car_2026`: 小车任务导航，沿 A-B-C-D-A 胶囊线路跑一圈。
- `2026_land`: 地面站 UI，发布任务启动指令并显示飞机/小车状态和位置。

三台设备联网运行时统一使用：

```bash
export ROS_DOMAIN_ID=99
```

## 场地坐标

所有协同话题使用米作为单位，坐标由题面 cm 转换得到：

| 点 | 坐标 m | 含义 |
| --- | --- | --- |
| H | `(0.75, 0.75)` | 无人机起降点 |
| A | `(1.50, 2.00)` | 小车起点/终点 |
| B | `(1.50, 3.50)` | 左上转弯点 |
| C | `(3.00, 3.50)` | 右上转弯点 |
| D | `(3.00, 2.00)` | 右下转弯点 |

巡航高度使用 `1.50m`。

## 统一 ROS2 话题协议

| 话题 | 类型 | 发布端 | 订阅端 | 用途 |
| --- | --- | --- | --- | --- |
| `/mission/command` | `std_msgs/msg/String` JSON | 地面站 | 飞机、小车 | 启动任务一/二 |
| `/drone/status` | `std_msgs/msg/String` JSON | 飞机 | 地面站 | 飞机任务阶段 |
| `/car/status` | `std_msgs/msg/String` JSON | 小车 | 地面站 | 小车任务阶段 |
| `/lidar_data` | `ros2_tools/msg/LidarPose` | 飞机 | 地面站、飞机内部 | 飞机定位 |
| `/carrier/lidar_pose` | `ros2_tools/msg/LidarPose` | 小车 | 飞机、地面站 | 小车/航母平台位姿 |
| `/drone/airdrop_cmd` | `std_msgs/msg/String` JSON | 飞机 | 抛投机构节点 | 任务一抛投触发 |

任务启动命令示例：

```bash
ros2 topic pub /mission/command std_msgs/msg/String \
  '{data: "{\"command\":\"start\",\"task\":1}"}' -1
```

其中 `task=1` 为抛投任务，`task=2` 为动态起降任务。

## 飞机端 `2026_elec`

新增能力：

- `DroneHAL` 订阅 `/mission/command`，收到地面站 `start` 后才允许任务开始。
- `DroneHAL` 发布 `/drone/status`，供地面站文字显示。
- `DroneHAL` 发布 `/drone/airdrop_cmd`，供后续抛投机构节点订阅。
- `MissionExecutor` 根据任务编号执行两套流程。

任务一当前流程：

1. 飞到 H 点上方 `(0.75, 0.75, 1.50)`。
2. 稳定悬停 `3s`。
3. 跟随 `/carrier/lidar_pose` 建立伴飞。
4. 发布 `/drone/airdrop_cmd` 的 `release` 命令。
5. 返回 H 点上方并降落。

任务二当前流程：

1. 飞到 H 点上方 `(0.75, 0.75, 1.50)`。
2. 稳定悬停 `3s`。
3. 跟随小车寻找平台。
4. 定高跟随下降到 `0.15m` 作为动态落车阶段。
5. 在车上停留 `5s`。
6. 复飞到 `1.50m`。
7. 返回 H 点上方并降落。

启动：

```bash
cd ~/ros2/2026_elec
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch core_2026 core_launch.py carrier_pose_topic:=/carrier/lidar_pose
```

## 小车端 `car_2026`

新增能力：

- Python 任务导航节点 `waypoint_nav_py` 接收 `/mission/command`。
- 路径改为题面 A-B-C-D-A 胶囊路线。
- 小车发布 `/carrier/lidar_pose`，格式与飞机端 `ros2_tools/msg/LidarPose` 对齐。
- 小车发布 `/car/status` 给地面站显示。

小车坐标转换：

- 首帧 VIO 位姿作为本地原点。
- 默认映射到场地 A 点 `(1.50, 2.00)`。
- 默认初始航向为 `+Y`，即从 A 指向 B。

启动：

```bash
cd ~/ros2/car_2026
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch line_follower navigate.launch.py
```

## 地面站 `2026_land`

新增内容：

- 独立 `ros2_tools/msg/LidarPose` 消息包。
- `land_station` pygame UI。
- 按 `1` 启动任务一，按 `2` 启动任务二。
- 实时显示场地、H/A/B/C/D、飞机位置、小车位置、双方状态。

启动：

```bash
cd ~/ros2/2026_land
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch land_station ground_station.launch.py
```

如果系统缺少 pygame：

```bash
python3 -m pip install pygame
```

## 构建验证

本次已分别验证：

```bash
cd ~/ros2/2026_elec
colcon build --packages-select core_2026 --allow-overriding messages ros2_tools --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

cd ~/ros2/car_2026
colcon build --packages-select ros2_tools line_follower --allow-overriding ros2_tools --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

cd ~/ros2/2026_land
colcon build --packages-select ros2_tools land_station --allow-overriding ros2_tools
```

三端均构建通过。

## 当前边界

- `/drone/airdrop_cmd` 已发布抛投触发命令，但实际舵机/电磁铁释放节点需要后续接入该话题。
- 任务二的动态落车目前用“跟随小车位姿并下降到低高度”的控制策略，实际落车高度需要结合平台高度、PX4降落模式和安全测试继续标定。
- 小车定位默认按首帧 VIO 对齐 A 点；如果实车 SLAM 已经输出场地绝对坐标，可把转换参数调成零偏或进一步改为直通。
