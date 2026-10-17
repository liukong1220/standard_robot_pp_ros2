#ifndef STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_ADDON_HPP_
#define STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_ADDON_HPP_

#include <cstdint>

#include "standard_robot_pp_ros2/packet_typedef.hpp"

namespace standard_robot_pp_ros2
{
// ============================================================
// 串口融合改造新增包定义（v1.0，详见 Start/doc/协议规格书.md）
// 规则：不动现有 14 种包的任何字段；仅新增以下两种。
// 硬约束：所有帧总长 <= 64 字节。
// ============================================================

// Receive 新增 ID（现有接收 ID 用到 0x0D）
const uint8_t ID_GIMBAL_STATE = 0x0E;
// Send 新增 ID（现有发送仅 0x01）
const uint8_t ID_GIMBAL_CMD = 0x02;

// ---------- mode 扩展值（仲裁记录：导航优先） ----------
// 仅用于 ID_GIMBAL_STATE.mode，不与现有任何包的 mode 混用。
// 0~3 沿用原视觉 GimbalToVision.mode 语义；4/5/6 为融合新增。
enum GimbalStateMode : uint8_t
{
  GIMBAL_MODE_IDLE = 0,          // 空闲
  GIMBAL_MODE_AUTO_AIM = 1,      // 自瞄
  GIMBAL_MODE_SMALL_BUFF = 2,    // 小符
  GIMBAL_MODE_BIG_BUFF = 3,      // 大符
  GIMBAL_MODE_NAV_OVERRIDE = 4,  // 融合新增：导航接管，电控屏蔽视觉指令
  GIMBAL_MODE_CONFLICT = 5,      // 融合新增：两路指令同时有效，视觉降级
  GIMBAL_MODE_RESERVED = 6,      // 融合预留：视觉按 1 回退并告警
};

// 电控 -> 上位机：云台状态（替代原 sp_vision25 GimbalToVision，42B 裸帧）
// data 39B：mode(1) q[4](16) yaw/vel pitch/vel(16) bullet_speed(4) bullet_count(2)
struct __attribute__((packed)) ReceiveGimbalStateData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct __attribute__((packed)) data
  {
    uint8_t mode;      // 见 GimbalStateMode；角度一律弧度
    float q[4];        // 云台姿态四元数，wxyz 顺序
    float yaw;         // rad
    float yaw_vel;     // rad/s
    float pitch;       // rad
    float pitch_vel;   // rad/s
    float bullet_speed;  // m/s
    uint16_t bullet_count;  // 累计发弹数
  } data;

  uint16_t crc;
};
static_assert(sizeof(ReceiveGimbalStateData) == 49, "ID_GIMBAL_STATE must be 49 bytes");
static_assert(sizeof(ReceiveGimbalStateData) <= 64, "frame exceeds 64-byte USB limit");

// 上位机 -> 电控：自瞄控制指令（替代原 sp_vision25 VisionToGimbal，28B 裸帧）
// data 25B：mode(1) + 6 x float(24)
struct __attribute__((packed)) SendGimbalCmdData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct __attribute__((packed)) data
  {
    uint8_t mode;        // 0-不控制 1-控制不开火 2-控制且开火
    float yaw;           // rad
    float yaw_vel;       // rad/s
    float yaw_acc;       // rad/s^2
    float pitch;         // rad
    float pitch_vel;     // rad/s
    float pitch_acc;     // rad/s^2
  } data;

  uint16_t crc;
};
static_assert(sizeof(SendGimbalCmdData) == 35, "ID_GIMBAL_CMD must be 35 bytes");
static_assert(sizeof(SendGimbalCmdData) <= 64, "frame exceeds 64-byte USB limit");

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_ADDON_HPP_