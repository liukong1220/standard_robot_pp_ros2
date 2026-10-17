#ifndef STANDARD_ROBOT_PP_ROS2__GIMBAL_BRIDGE_ADDON_HPP_
#define STANDARD_ROBOT_PP_ROS2__GIMBAL_BRIDGE_ADDON_HPP_

// 串口融合改造：导航节点新增的视觉桥接逻辑。
// 使用方式：本文件定义独立类 GimbalBridgeAddon，在
// standard_robot_pp_ros2.hpp 中以成员形式持有（见 nav_bridge/README.md 步骤 2），
// 不要把本文件内容复制进 StandardRobotPpRos2Node 类定义内部。

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef_addon.hpp"

namespace standard_robot_pp_ros2
{

// ============================================================
// 新增话题定义
// ============================================================
// /serial/gimbal_state : std_msgs/Float64MultiArray（电控->视觉）
//   索引: [0]=mode [1..4]=q_w,q_x,q_y,q_z [5]=yaw [6]=yaw_vel
//          [7]=pitch [8]=pitch_vel [9]=bullet_speed [10]=bullet_count
//   维度: data.size()==11；无有效帧时不发布。
//   说明：header 用 ROS now()（到达时刻），视觉 slerp 依赖到达时序，
//         不透传下位机 time_stamp（协议字段仍按原样保存）。
//
// /serial/gimbal_cmd : std_msgs/Float64MultiArray（视觉->电控）
//   索引: [0]=mode [1]=yaw [2]=yaw_vel [3]=yaw_acc
//          [4]=pitch [5]=pitch_vel [6]=pitch_acc
//   维度: data.size()==7；由独立线程按最新值组包发送。
//   话题内数值一律弧度/弧度每秒/弧度每二次方秒。
//
// 话题类型选用 Float64MultiArray 的原因：视觉仓库已有 sp_msgs，
// 但导航侧为减少接口包耦合，先用标准消息承载；后续如需强类型
// 可在 sp_msgs 中定义 GimbalStateMsg/GimbalCmdMsg 替换，字段不变。

class GimbalBridgeAddon
{
public:
  // 视觉指令失联判定窗口，与协议规格书 §3.2 电控端 ≥100ms 超时语义对齐：
  // 超过该时间无新话题帧则停发（电控随之进入失联处理），绝不无限重发旧帧。
  static constexpr int kGimbalCmdTimeoutMs = 100;
  static constexpr uint8_t kModeTrackNoFire = 1;
  static constexpr uint8_t kModeTrackAndFire = 2;

  using SerialSendFn = std::function<void(const std::vector<uint8_t> &)>;
  using FirePermittedFn = std::function<bool()>;

  // node: 宿主 StandardRobotPpRos2Node（this）。
  // serial_send_fn 必须在构造时注入：发送线程在构造函数中已启动，
  // 事后调用 setSerialSendFn 会对 std::function 产生并发读写（UB）。
  GimbalBridgeAddon(
    rclcpp::Node & node, SerialSendFn serial_send_fn, FirePermittedFn fire_permitted_fn)
  : node_(node),
    serial_send_fn_(std::move(serial_send_fn)),
    fire_permitted_fn_(std::move(fire_permitted_fn)),
    gimbal_state_pub_(node.create_publisher<std_msgs::msg::Float64MultiArray>(
      "serial/gimbal_state", 10)),
    gimbal_cmd_sub_(node.create_subscription<std_msgs::msg::Float64MultiArray>(
      "serial/gimbal_cmd", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(gimbal_cmd_mutex_);
        latest_gimbal_cmd_ = *msg;
        latest_gimbal_cmd_time_ = std::chrono::steady_clock::now();
      }))
  {
    // 发送线程：独立于现有 sendData 的 5ms 周期，按视觉节拍转发。
    // 不与 SendRobotCmdData 复用线程，避免导航发送节拍影响视觉指令延迟。
    gimbal_cmd_send_thread_ = std::thread([this]() { gimbalCmdSendLoop(); });
  }

  ~GimbalBridgeAddon()
  {
    gimbal_cmd_running_ = false;
    if (gimbal_cmd_send_thread_.joinable()) gimbal_cmd_send_thread_.join();
  }

  // 供 receiveData() 的 switch 新增分支调用：
  // case ID_GIMBAL_STATE: gimbal_bridge_.publishGimbalState(gimbal_state_data); break;
  void publishGimbalState(const ReceiveGimbalStateData & d)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(11);
    msg.data.push_back(static_cast<double>(d.data.mode));
    for (int i = 0; i < 4; ++i) msg.data.push_back(static_cast<double>(d.data.q[i]));
    msg.data.push_back(static_cast<double>(d.data.yaw));
    msg.data.push_back(static_cast<double>(d.data.yaw_vel));
    msg.data.push_back(static_cast<double>(d.data.pitch));
    msg.data.push_back(static_cast<double>(d.data.pitch_vel));
    msg.data.push_back(static_cast<double>(d.data.bullet_speed));
    msg.data.push_back(static_cast<double>(d.data.bullet_count));
    gimbal_state_pub_->publish(msg);
  }

private:
  void gimbalCmdSendLoop()
  {
    // 帧头只需初始化一次；time_stamp 每包刷新；CRC16 每包重算。
    send_cmd_.frame_header.sof = SOF_SEND;
    send_cmd_.frame_header.id = ID_GIMBAL_CMD;
    send_cmd_.frame_header.len = sizeof(SendGimbalCmdData) - 6;

    const auto period = std::chrono::milliseconds(5);  // 上限 200Hz 转发
    auto next_tick = std::chrono::steady_clock::now();
    while (rclcpp::ok() && gimbal_cmd_running_) {
      next_tick += period;
      bool has_cmd = false;
      {
        std::lock_guard<std::mutex> lock(gimbal_cmd_mutex_);
        const bool fresh =
          latest_gimbal_cmd_time_ != std::chrono::steady_clock::time_point{} &&
          (std::chrono::steady_clock::now() - latest_gimbal_cmd_time_) <
            std::chrono::milliseconds(kGimbalCmdTimeoutMs);
        if (latest_gimbal_cmd_.data.size() == 7 && fresh) {
          bool finite = true;
          for (const auto value : latest_gimbal_cmd_.data) {
            finite = finite && std::isfinite(value);
          }
          if (finite) {
            has_cmd = true;
            send_cmd_.data.mode = static_cast<uint8_t>(latest_gimbal_cmd_.data[0]);
            send_cmd_.data.yaw = static_cast<float>(latest_gimbal_cmd_.data[1]);
            send_cmd_.data.yaw_vel = static_cast<float>(latest_gimbal_cmd_.data[2]);
            send_cmd_.data.yaw_acc = static_cast<float>(latest_gimbal_cmd_.data[3]);
            send_cmd_.data.pitch = static_cast<float>(latest_gimbal_cmd_.data[4]);
            send_cmd_.data.pitch_vel = static_cast<float>(latest_gimbal_cmd_.data[5]);
            send_cmd_.data.pitch_acc = static_cast<float>(latest_gimbal_cmd_.data[6]);
          }
        }
      }
      if (has_cmd && serial_send_fn_) {
        if (send_cmd_.data.mode > kModeTrackAndFire) {
          send_cmd_.data.mode = 0;
        }
        if (send_cmd_.data.mode == kModeTrackAndFire &&
          !(fire_permitted_fn_ && fire_permitted_fn_()))
        {
          send_cmd_.data.mode = kModeTrackNoFire;
        }
        send_cmd_.time_stamp = static_cast<uint32_t>(
          node_.now().nanoseconds() / 1000000ULL);
        // 帧头 CRC8（内容固定，但保持与现有 sendData 相同的每包计算方式，
        // 防止结构体内存被意外改动后校验失配）
        crc8::append_CRC8_check_sum(
          reinterpret_cast<uint8_t *>(&send_cmd_), sizeof(HeaderFrame));
        // 整包 CRC16
        crc16::append_CRC16_check_sum(
          reinterpret_cast<uint8_t *>(&send_cmd_), sizeof(SendGimbalCmdData));
        serial_send_fn_(toVector(send_cmd_));
      }
      std::this_thread::sleep_until(next_tick);
    }
  }

private:
  rclcpp::Node & node_;
  std::function<void(const std::vector<uint8_t> &)> serial_send_fn_;
  FirePermittedFn fire_permitted_fn_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_sub_;

  std::mutex gimbal_cmd_mutex_;
  std_msgs::msg::Float64MultiArray latest_gimbal_cmd_;
  // 话题最后一帧到达时刻：用于失联检测（对应电控端 ≥100ms 超时语义）。
  std::chrono::steady_clock::time_point latest_gimbal_cmd_time_{};
  SendGimbalCmdData send_cmd_{};
  std::thread gimbal_cmd_send_thread_;
  std::atomic<bool> gimbal_cmd_running_{true};
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__GIMBAL_BRIDGE_ADDON_HPP_
