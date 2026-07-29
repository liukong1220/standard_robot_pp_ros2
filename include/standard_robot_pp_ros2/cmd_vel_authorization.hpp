// Copyright 2026
//
// 串口底盘出口的授权与归零判据（P6 一.2/一.3）。
//
// 这里只放纯逻辑，不碰串口、不碰 ROS，目的是让下面四条判据可以被单元测试直接覆盖：
//
//  1. 「授权归零」必须立刻生效，不得被 `enable_transient_zero_cmd_hold` 的
//     瞬时零保持窗口吞掉。授权归零包括：`ExecutionCommand.mode == MODE_STOP`、
//     `emergency_stop == true`、串口断连、看门狗超时、以及缺少执行授权。
//  2. 「通信抖动导致的瞬时零」才允许保持最近一次非零命令，且窗口上界是
//     `transient_zero_cmd_hold_timeout`，并且只在已授权且未急停时成立。
//  3. 串口断连后授权立即失效；重连成功不自动恢复旧命令，必须由一条
//     `command_sequence` 严格更大、且时间戳新鲜的 `MODE_EXECUTE` 重新授权。
//  4. 单独的 `emergency_stop == false` 不构成授权，不能恢复运动。
//
// 时序关系（全部以 steady clock 为基准）：
//
//   MPC 控制周期     $T_{\mathrm{mpc}} = 1 / 20\ \mathrm{Hz} = 50\ \mathrm{ms}$
//   瞬时零保持窗口   $T_{\mathrm{hold}} = 50\ \mathrm{ms} = 1\,T_{\mathrm{mpc}}$
//   底盘看门狗       $T_{\mathrm{wd}} = 300\ \mathrm{ms} = 6\,T_{\mathrm{mpc}}$
//   授权/急停租约上界 $T_{\mathrm{lease}} = 500\ \mathrm{ms}$
//
// 必须满足 $T_{\mathrm{hold}} \le T_{\mathrm{mpc}} \cdot 2 < T_{\mathrm{wd}}
// \le T_{\mathrm{lease}}$：看门狗不得比上游租约更宽松，否则「上游租约超时 ->
// 全链归零」的上界推导在底盘这一级失效。

#ifndef STANDARD_ROBOT_PP_ROS2__CMD_VEL_AUTHORIZATION_HPP_
#define STANDARD_ROBOT_PP_ROS2__CMD_VEL_AUTHORIZATION_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>

namespace standard_robot_pp_ros2
{

/// 出口决策的原因码，用于日志与测试断言。
enum class CmdVelGateReason : uint8_t {
  /// 直接下发输入命令。
  FORWARD = 0,
  /// 判定为通信抖动的瞬时零，保持最近一次非零命令。
  HOLD_TRANSIENT_ZERO = 1,
  /// 授权侧要求归零（ExecutionCommand STOP 或 emergency_stop）。
  AUTHORIZED_ZERO = 2,
  /// 尚未获得执行授权，或授权已失效。
  NO_AUTHORIZATION = 3,
  /// cmd_vel 断流超过看门狗窗口。
  WATCHDOG_TIMEOUT = 4,
  /// 串口断连。
  LINK_DOWN = 5,
};

struct CmdVelGateConfig
{
  /// 是否要求 `ExecutionCommand` 授权后才允许下发非零速度。
  /// Nav2-free 官方 profile 必须为 true；Nav2 对照 profile 没有该话题，为 false。
  bool require_execution_authorization = false;
  bool enable_transient_zero_cmd_hold = true;
  std::chrono::milliseconds transient_zero_cmd_hold_timeout{50};
  std::chrono::milliseconds cmd_vel_watchdog_timeout{300};
  /// 授权命令可接受的最大时间戳年龄。
  /// 用于拒绝 transient-local 重投的旧样本。
  std::chrono::milliseconds execution_command_timeout{500};
  double linear_epsilon = 1e-3;
  double angular_epsilon = 1e-3;
};

struct CmdVelGateOutput
{
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  /// 写入协议 `speed_vector.stop`。任何授权归零、断连、看门狗都必须置 true。
  bool stop = true;
  CmdVelGateReason reason = CmdVelGateReason::NO_AUTHORIZATION;

  bool isZero() const { return vx == 0.0 && vy == 0.0 && wz == 0.0; }
};

/// 出口授权门。所有方法都由调用者在同一把锁下串行调用。
class CmdVelAuthorizationGate
{
public:
  explicit CmdVelAuthorizationGate(const CmdVelGateConfig & config = CmdVelGateConfig())
  : config_(config)
  {
  }

  void setConfig(const CmdVelGateConfig & config) { config_ = config; }
  const CmdVelGateConfig & config() const { return config_; }

  /// 收到一条执行授权。返回 true 表示该样本被接受（无论 EXECUTE 还是 STOP）。
  ///
  /// `age` 是命令头时间戳到现在的年龄，用于拒绝重启后被重投的 latched 旧样本；
  /// `sequence` 必须严格单调递增，否则视为重放并拒绝。
  bool onExecutionCommand(
    bool execute, uint64_t sequence, uint64_t localization_epoch, uint64_t map_generation,
    std::chrono::milliseconds age)
  {
    if (age > config_.execution_command_timeout) {
      return false;
    }
    if (has_command_sequence_ && sequence <= last_command_sequence_) {
      return false;
    }
    has_command_sequence_ = true;
    last_command_sequence_ = sequence;
    last_localization_epoch_ = localization_epoch;
    last_map_generation_ = map_generation;
    if (execute) {
      // 授权只能由 EXECUTE 建立。
      // 断连期间收到的 EXECUTE 也不算，链路先要恢复。
      execution_stop_ = false;
      authorized_ = link_up_;
    } else {
      execution_stop_ = true;
      authorized_ = false;
      has_nonzero_cmd_ = false;
    }
    return true;
  }

  /// `true` 立即进入授权归零；`false` 只清除急停标志，不恢复授权。
  void onEmergencyStop(bool active)
  {
    emergency_stop_ = active;
    if (active) {
      authorized_ = false;
      has_nonzero_cmd_ = false;
    }
  }

  /// 串口断连：授权立即失效，缓存的非零命令作废。
  void onSerialLinkDown()
  {
    link_up_ = false;
    authorized_ = false;
    has_nonzero_cmd_ = false;
    has_cmd_ = false;
  }

  /// 串口重连成功：只恢复链路，不恢复授权。
  void onSerialLinkUp() { link_up_ = true; }

  bool authorized() const { return authorized_; }
  bool linkUp() const { return link_up_; }
  uint64_t lastCommandSequence() const { return last_command_sequence_; }
  uint64_t lastLocalizationEpoch() const { return last_localization_epoch_; }
  uint64_t lastMapGeneration() const { return last_map_generation_; }

  /// 处理一帧 cmd_vel。`now` 必须是 steady clock 时刻。
  CmdVelGateOutput onCmdVel(
    double vx, double vy, double wz, std::chrono::steady_clock::time_point now)
  {
    has_cmd_ = true;
    last_cmd_time_ = now;

    const bool is_zero = std::abs(vx) <= config_.linear_epsilon &&
                         std::abs(vy) <= config_.linear_epsilon &&
                         std::abs(wz) <= config_.angular_epsilon;

    if (!link_up_) {
      has_nonzero_cmd_ = false;
      return zeroOutput(CmdVelGateReason::LINK_DOWN);
    }
    if (emergency_stop_ || execution_stop_) {
      has_nonzero_cmd_ = false;
      return zeroOutput(CmdVelGateReason::AUTHORIZED_ZERO);
    }
    if (config_.require_execution_authorization && !authorized_) {
      has_nonzero_cmd_ = false;
      return zeroOutput(CmdVelGateReason::NO_AUTHORIZATION);
    }

    if (!is_zero) {
      last_nonzero_vx_ = vx;
      last_nonzero_vy_ = vy;
      last_nonzero_wz_ = wz;
      last_nonzero_time_ = now;
      has_nonzero_cmd_ = true;
      CmdVelGateOutput output;
      output.vx = vx;
      output.vy = vy;
      output.wz = wz;
      output.stop = false;
      output.reason = CmdVelGateReason::FORWARD;
      return output;
    }

    if (
      config_.enable_transient_zero_cmd_hold && has_nonzero_cmd_ &&
      config_.transient_zero_cmd_hold_timeout.count() > 0) {
      const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_nonzero_time_);
      if (elapsed.count() >= 0 && elapsed <= config_.transient_zero_cmd_hold_timeout) {
        CmdVelGateOutput output;
        output.vx = last_nonzero_vx_;
        output.vy = last_nonzero_vy_;
        output.wz = last_nonzero_wz_;
        output.stop = false;
        output.reason = CmdVelGateReason::HOLD_TRANSIENT_ZERO;
        return output;
      }
    }

    has_nonzero_cmd_ = false;
    CmdVelGateOutput output;
    output.stop = false;
    output.reason = CmdVelGateReason::FORWARD;
    return output;
  }

  /// 发送线程每拍调用。返回 `true` 表示必须用返回的输出覆盖当前缓存命令。
  bool tick(std::chrono::steady_clock::time_point now, CmdVelGateOutput & output)
  {
    if (!link_up_) {
      output = zeroOutput(CmdVelGateReason::LINK_DOWN);
      return true;
    }
    if (emergency_stop_ || execution_stop_) {
      output = zeroOutput(CmdVelGateReason::AUTHORIZED_ZERO);
      return true;
    }
    if (config_.require_execution_authorization && !authorized_) {
      output = zeroOutput(CmdVelGateReason::NO_AUTHORIZATION);
      return true;
    }
    if (has_cmd_ && config_.cmd_vel_watchdog_timeout.count() > 0) {
      const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_time_);
      if (elapsed > config_.cmd_vel_watchdog_timeout) {
        has_nonzero_cmd_ = false;
        output = zeroOutput(CmdVelGateReason::WATCHDOG_TIMEOUT);
        return true;
      }
    }
    return false;
  }

private:
  static CmdVelGateOutput zeroOutput(CmdVelGateReason reason)
  {
    CmdVelGateOutput output;
    output.stop = true;
    output.reason = reason;
    return output;
  }

  CmdVelGateConfig config_;

  bool link_up_ = true;
  bool authorized_ = false;
  bool emergency_stop_ = false;
  bool execution_stop_ = false;

  bool has_command_sequence_ = false;
  uint64_t last_command_sequence_ = 0;
  uint64_t last_localization_epoch_ = 0;
  uint64_t last_map_generation_ = 0;

  bool has_cmd_ = false;
  bool has_nonzero_cmd_ = false;
  double last_nonzero_vx_ = 0.0;
  double last_nonzero_vy_ = 0.0;
  double last_nonzero_wz_ = 0.0;
  std::chrono::steady_clock::time_point last_cmd_time_{};
  std::chrono::steady_clock::time_point last_nonzero_time_{};
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__CMD_VEL_AUTHORIZATION_HPP_
