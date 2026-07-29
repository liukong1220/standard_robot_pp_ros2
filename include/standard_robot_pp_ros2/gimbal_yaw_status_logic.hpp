// Copyright 2026
//
// 实车云台 yaw 反馈到 `GimbalYawStatus` 的判据（P6 零.3）。
//
// 这里只放纯逻辑，不碰 ROS、不碰 TF，目的是让「什么情况下才允许 ack」可以被
// 单元测试直接覆盖。禁止伪造 ack，因此本判据的每一条都必须由实测量支撑：
//
//  1. `tf_healthy` 只能来自实测：串口 `serial/gimbal_joint_state` 新鲜，并且
//     `odom->gimbal_yaw_odom` 与 `odom->base_footprint` 两条 TF 查询都成功且
//     新鲜。任何一项缺失都必须是 `false`，不得写死 `true`。
//  2. `yaw_authority` 只有在物理前置条件实测满足时才回报请求的模式；否则一律
//     回报 `HOLD_SAFE_STOP`。这是「拒绝授权」，不是「延迟授权」。
//  3. `BODY_YAW_FOLLOW` 在实车上没有车体 yaw 交接机构，本判据结构性拒绝：
//     即使请求该模式，也只回报 `HOLD_SAFE_STOP`。
//  4. `locked` 只能来自实测云台 yaw 角速度：$|\dot{\psi}| \le
//     \omega_{\mathrm{lock}}$ 且连续保持 $T_{\mathrm{dwell}}$ 才为 `true`。
//  5. `request_sequence` 只回报已经收到过的请求序号，且只接受严格递增的请求；
//     未收到任何请求时回报 `0`，让上游的精确序号比较自然不成立。
//
// 时序关系（steady clock）：
//
//   云台反馈新鲜度上界 $T_{\mathrm{joint}}$，默认 $0.1\ \mathrm{s}$
//   TF 新鲜度上界      $T_{\mathrm{tf}}$，默认 $0.2\ \mathrm{s}$
//   状态发布周期       $T_{\mathrm{pub}} = 0.02\ \mathrm{s}$
//   上游云台租约       $T_{\mathrm{lease}} = 0.5\ \mathrm{s}$
//
// 必须满足 $T_{\mathrm{pub}} < T_{\mathrm{joint}} \le T_{\mathrm{tf}} <
// T_{\mathrm{lease}}$：本节点判定不新鲜的时刻必须早于上游租约超时，否则上游会
// 先看到「一直有效」的旧 ack。

#ifndef STANDARD_ROBOT_PP_ROS2__GIMBAL_YAW_STATUS_LOGIC_HPP_
#define STANDARD_ROBOT_PP_ROS2__GIMBAL_YAW_STATUS_LOGIC_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>

namespace standard_robot_pp_ros2
{

/// 与 `ats_navigation_interfaces/msg/GimbalYawStatus` 的常量保持一致。
enum class YawAuthority : uint8_t {
  HOLD_SAFE_STOP = 0,
  GIMBAL_COMPENSATED = 1,
  BODY_YAW_FOLLOW = 2,
};

/// 拒绝 ack 的原因码，用于日志与测试断言。
enum class GimbalAckReason : uint8_t {
  /// 按请求回报授权。
  ACK_GRANTED = 0,
  /// 尚未收到任何 YawAuthorityRequest。
  NO_REQUEST = 1,
  /// 云台关节反馈缺失或过期。
  FEEDBACK_STALE = 2,
  /// TF 查询失败或过期。
  TF_UNHEALTHY = 3,
  /// 请求 BODY_YAW_FOLLOW：实车无车体 yaw 交接机构，结构性拒绝。
  BODY_YAW_FOLLOW_FORBIDDEN = 4,
  /// 请求要求锁定，但实测云台仍在转动。
  NOT_LOCKED = 5,
};

struct GimbalYawStatusConfig
{
  std::chrono::milliseconds joint_state_timeout{100};
  std::chrono::milliseconds tf_timeout{200};
  /// 判定「云台已停住」的角速度上界，单位 $\mathrm{rad/s}$。
  double lock_rate_threshold = 0.05;
  /// 角速度持续低于上界的最短时间。
  std::chrono::milliseconds lock_dwell{100};
  /// 实车恒为 false。留作参数只是为了让「被拒绝」这件事可显式配置与测试。
  bool allow_body_yaw_follow = false;
};

struct GimbalYawStatusOutput
{
  uint64_t request_sequence = 0;
  YawAuthority yaw_authority = YawAuthority::HOLD_SAFE_STOP;
  bool locked = false;
  bool tf_healthy = false;
  double gimbal_yaw = 0.0;
  double body_yaw = 0.0;
  double fake_yaw = 0.0;
  GimbalAckReason reason = GimbalAckReason::NO_REQUEST;
};

/// 云台 ack 判据。所有方法由调用者在同一把锁下串行调用。
class GimbalYawStatusEvaluator
{
public:
  explicit GimbalYawStatusEvaluator(const GimbalYawStatusConfig & config = GimbalYawStatusConfig())
  : config_(config)
  {
  }

  void setConfig(const GimbalYawStatusConfig & config) { config_ = config; }
  const GimbalYawStatusConfig & config() const { return config_; }

  /// 收到一条授权请求。返回 `true` 表示被接受；序号非严格递增一律拒绝。
  bool onRequest(uint64_t request_sequence, YawAuthority authority, bool require_lock)
  {
    if (has_request_ && request_sequence <= request_sequence_) {
      return false;
    }
    has_request_ = true;
    request_sequence_ = request_sequence;
    requested_authority_ = authority;
    require_lock_ = require_lock;
    return true;
  }

  /// 收到一帧实测云台反馈。`tf_ok` 必须来自真实 TF 查询结果。
  ///
  /// `gimbal_yaw` 是 `gimbal_yaw_odom` 在 odom 中的实测 yaw，
  /// `body_yaw` 是车体基座在 odom 中的实测 yaw，两者都不得由请求推导。
  void onFeedback(
    double gimbal_yaw, double body_yaw, bool tf_ok, std::chrono::steady_clock::time_point now)
  {
    if (has_feedback_) {
      const double dt =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - feedback_time_).count();
      if (dt > 0.0) {
        const double delta = normalizeAngle(gimbal_yaw - gimbal_yaw_);
        yaw_rate_ = delta / dt;
        has_yaw_rate_ = true;
      }
    }
    // fake_yaw 是 fake-yaw 方案的参考零位 $\psi_0$：取第一帧 TF 健康的实测
    // 云台 yaw，而不是任何配置常量，这样它与真实关节零位一致。
    if (!has_fake_yaw_ && tf_ok) {
      fake_yaw_ = gimbal_yaw;
      has_fake_yaw_ = true;
    }
    gimbal_yaw_ = gimbal_yaw;
    body_yaw_ = body_yaw;
    tf_ok_ = tf_ok;
    feedback_time_ = now;
    has_feedback_ = true;

    if (has_yaw_rate_ && std::abs(yaw_rate_) <= config_.lock_rate_threshold) {
      if (!still_since_valid_) {
        still_since_ = now;
        still_since_valid_ = true;
      }
    } else {
      still_since_valid_ = false;
    }
  }

  /// 计算当前应发布的状态。绝不因为「收到请求」就把授权写成请求值。
  GimbalYawStatusOutput evaluate(std::chrono::steady_clock::time_point now) const
  {
    GimbalYawStatusOutput output;
    output.request_sequence = has_request_ ? request_sequence_ : 0U;
    output.gimbal_yaw = gimbal_yaw_;
    output.body_yaw = body_yaw_;
    output.fake_yaw = fake_yaw_;

    const bool feedback_fresh =
      has_feedback_ && std::chrono::duration_cast<std::chrono::milliseconds>(
                         now - feedback_time_) <= config_.joint_state_timeout;
    const bool tf_fresh = tf_ok_ && has_feedback_ &&
                          std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - feedback_time_) <= config_.tf_timeout;
    output.tf_healthy = feedback_fresh && tf_fresh && has_fake_yaw_;

    output.locked = output.tf_healthy && still_since_valid_ &&
                    std::chrono::duration_cast<std::chrono::milliseconds>(now - still_since_) >=
                      config_.lock_dwell;

    if (!feedback_fresh) {
      output.reason = GimbalAckReason::FEEDBACK_STALE;
      return output;
    }
    if (!output.tf_healthy) {
      output.reason = GimbalAckReason::TF_UNHEALTHY;
      return output;
    }
    if (!has_request_) {
      output.reason = GimbalAckReason::NO_REQUEST;
      return output;
    }
    if (requested_authority_ == YawAuthority::BODY_YAW_FOLLOW && !config_.allow_body_yaw_follow) {
      // 实车没有车体 yaw 交接机构：回报 HOLD_SAFE_STOP，让上游因模式不匹配停车。
      output.reason = GimbalAckReason::BODY_YAW_FOLLOW_FORBIDDEN;
      return output;
    }
    if (require_lock_ && !output.locked) {
      output.reason = GimbalAckReason::NOT_LOCKED;
      return output;
    }
    output.yaw_authority = requested_authority_;
    output.reason = GimbalAckReason::ACK_GRANTED;
    return output;
  }

  bool hasRequest() const { return has_request_; }
  uint64_t requestSequence() const { return request_sequence_; }
  double yawRate() const { return yaw_rate_; }

private:
  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  GimbalYawStatusConfig config_;

  bool has_request_ = false;
  uint64_t request_sequence_ = 0;
  YawAuthority requested_authority_ = YawAuthority::HOLD_SAFE_STOP;
  bool require_lock_ = false;

  bool has_feedback_ = false;
  bool tf_ok_ = false;
  double gimbal_yaw_ = 0.0;
  double body_yaw_ = 0.0;
  bool has_fake_yaw_ = false;
  double fake_yaw_ = 0.0;
  std::chrono::steady_clock::time_point feedback_time_{};

  bool has_yaw_rate_ = false;
  double yaw_rate_ = 0.0;
  bool still_since_valid_ = false;
  std::chrono::steady_clock::time_point still_since_{};
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__GIMBAL_YAW_STATUS_LOGIC_HPP_
