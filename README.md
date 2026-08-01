# standard_robot_pp_ros2

ATS 哨兵上位机与下位机/裁判系统之间的 ROS 2 串口桥。它是实机底盘命令的
最终出口，同时发布云台关节、IMU、机器人状态和裁判系统数据。

> Nav2-free 正式入口要求 `ExecutionCommand` 授权；包内默认配置仍服务于
> Nav2 对照 profile，因此 `require_execution_authorization` 默认是 `false`。

## 目录

- [功能模块](#功能模块)
- [依赖](#依赖)
- [Quick Start](#quick-start)
- [启动节点](#启动节点)
- [接口](#接口)
- [安全与时序](#安全与时序)
- [配置文件](#配置文件)
- [数据流](#数据流)
- [测试与验证边界](#测试与验证边界)
- [参考与致谢](#参考与致谢)

## 功能模块

| 模块 | 说明 |
| :--- | :--- |
| 串口协议 | 解析下位机/裁判系统数据并编码控制帧 |
| 底盘出口 | 接收 `/cmd_vel`，写入车体系 `speed_vector[vx, vy, wz]` |
| 授权门 | 校验 `ExecutionCommand`、急停、串口状态和 watchdog |
| 云台桥 | `cmd_gimbal` 到关节指令，发布三自由度反馈 |
| yaw 状态桥 | 结合 joint/TF 发布 `/gimbal/yaw_status` |
| 裁判系统 | 发布比赛、血量、事件、RFID、机器人状态等消息 |

本包不拥有路径规划、速度坐标变换或 MPC。Nav2-free 下 MPC 必须已经输出车体系
速度，串口层只做授权、限时保持、归零和协议转换。

## 依赖

- ROS 2 Humble
- `rclcpp`、`serial_driver`、ASIO
- `ats_rm_interfaces`、`ats_navigation_interfaces`
- `geometry_msgs`、`sensor_msgs`、`std_msgs`、`tf2_ros`

## Quick Start

```bash
cd /home/ats/ATS_2026_snetry_test
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src \
  --packages-up-to standard_robot_pp_ros2 \
  --parallel-workers 1 --symlink-install
source install/setup.bash
```

## 启动节点

单包入口：

```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
  params_file:=src/standard_robot_pp_ros2/config/standard_robot_pp_ros2.yaml
```

实机正常部署由 `ats_sentry_bringup` 启动，并传入根仓
`src/ats_sentry_bringup/params/node_params.yaml`。单包默认配置与正式总入口的授权
开关不同，调试时必须先确认实际加载文件。

主要节点：

- `standard_robot_pp_ros2_node`
- `gimbal_manager_node`
- `gimbal_yaw_status_bridge`，由外层在需要真实云台 ack 时启动

## 接口

### 底盘与安全输入

| Topic | 类型 | 说明 |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | 车体系 `[vx, vy, wz]` |
| `/planner/execution_command` | `ats_navigation_interfaces/ExecutionCommand` | 正式链唯一非零执行授权 |
| `/planner/emergency_stop` | `std_msgs/Bool` | `true` 立即归零并撤销授权 |
| `decision/robot_mode` | `example_interfaces/UInt8` | 写入协议 `speed_vector.mode` |
| `cmd_gimbal` | `ats_rm_interfaces/GimbalCmd` | 云台命令 |

### 主要输出

| Topic | 说明 |
| :--- | :--- |
| `serial/imu` | 下位机 IMU，frame 为 `gimbal_pitch` |
| `serial/gimbal_joint_state` | `gimbal_yaw_odom_joint`、`gimbal_yaw_joint`、`gimbal_pitch_joint` |
| `serial/robot_motion` | 下位机反馈的车体速度 |
| `serial/robot_state_info` | 机器人硬件类型与状态 |
| `referee/*` | 比赛、血量、事件、位置、RFID、buff 等 |
| `/gimbal/yaw_status` | yaw authority ack 与 TF 健康状态 |

`/gimbal/yaw_status` 使用 reliable + transient-local QoS。其实机 publisher 必须唯一；
MuJoCo 的模拟 ack 与实机桥不能同时运行。

## 安全与时序

当前默认时序关系：

$$
T_{hold}=50\,\mathrm{ms}<T_{watchdog}=300\,\mathrm{ms}
\le T_{lease}=500\,\mathrm{ms}
$$

- 短暂全零保持只覆盖 1 个 20 Hz MPC 周期，减少偶发单帧冲击。
- cmd_vel 断流超过 300 ms 后主动归零并设置 `speed_vector.stop=true`。
- `ExecutionCommand STOP`、`emergency_stop=true`、串口断连或授权过期都归零。
- 单独收到 `emergency_stop=false` 只清急停标志，不恢复执行授权。
- 旧 command sequence、过期时间戳和旧 reference 不得重新放行。

节点会检查 watchdog 不宽于 execution lease，并在配置不满足时收紧参数。

## 配置文件

| 文件 | 用途 |
| :--- | :--- |
| `config/standard_robot_pp_ros2.yaml` | 单包/Nav2 对照默认配置 |
| `../ats_sentry_bringup/params/node_params.yaml` | 正式实机总入口实际配置 |

关键参数：

- `device_name`、`baud_rate`、`flow_control`、`parity`、`stop_bits`
- `require_execution_authorization`
- `execution_command_topic`、`emergency_stop_topic`
- `execution_command_timeout`、`cmd_vel_watchdog_timeout_ms`
- `enable_transient_zero_cmd_hold`、`transient_zero_cmd_hold_timeout_ms`
- `accept_legacy_two_axis_joint_state` 与 small-yaw 方向/offset

未来参数统一后正式实机值应只在根仓总 YAML 出现；包内 YAML 可保留为显式对照
或示例，但不能与总入口同时成为“正式权威”。

## 数据流

```text
Goal Manager atomic ExecutionCommand ----+
MPC body-frame /cmd_vel -----------------+-> CmdVelAuthorizationGate
emergency_stop + serial link + watchdog -+          |
                                                     v
                                  speed_vector[vx, vy, wz, mode, stop]
                                                     |
                                                     v
                                               serial firmware
```

云台状态链：

```text
serial/gimbal_joint_state + odom TF + yaw authority request
  -> gimbal_yaw_status_bridge
  -> /gimbal/yaw_status
  -> Goal Manager + MPC
```

禁止新增第二个 `base_footprint -> base_link` TF publisher。fake-yaw 关闭时仍需
保留 `gimbal_yaw_odom -> gimbal_yaw_fake` 零旋转兼容 TF，直到所有 consumer
完成 frame 迁移。

## 测试与验证边界

```bash
colcon test --base-paths src --packages-select standard_robot_pp_ros2
colcon test-result --test-result-base build/standard_robot_pp_ros2 --verbose
```

实机/HIL 还必须测量断流、STOP、急停、串口拔插和旧授权恢复，观察 `/cmd_vel`
以及下位机实际 `speed_vector.stop`，不能只看上游 topic。

- **已验证**：README 中 topic、默认时序和授权逻辑由源码/配置静态交叉核对。
- **未验证**：本轮未连接串口、未做抬轮 HIL 或落地实车测试。
- **残余风险**：`max_wheel_acceleration` 等动力学值仍需台架标定；真实固件单位和滚动半径需同口径复核。

工作区级说明见
[视觉与串口桥说明](../../docs/视觉与串口桥说明.md)。

## 参考与致谢

串口基础设施使用 ROS 2 `serial_driver`/ASIO，并延续 StandardRobot++ 协议适配。
具体版权、协议兼容和许可证以仓内源码与依赖声明为准。
