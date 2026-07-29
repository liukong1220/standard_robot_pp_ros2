// Copyright 2026
//
// 实车云台 ack 判据测试（P6 零.3）。覆盖：
//  - 无请求、反馈过期、TF 不健康时一律 HOLD_SAFE_STOP 且 tf_healthy=false
//  - BODY_YAW_FOLLOW 被结构性拒绝
//  - require_lock 时必须实测停住并保持 dwell 才 ack
//  - request_sequence 只回报已收到的请求，且非严格递增被拒
//  - tf_healthy 只能来自实测，不得写死

#include <gtest/gtest.h>

#include <chrono>

#include "standard_robot_pp_ros2/gimbal_yaw_status_logic.hpp"

using standard_robot_pp_ros2::GimbalAckReason;
using standard_robot_pp_ros2::GimbalYawStatusConfig;
using standard_robot_pp_ros2::GimbalYawStatusEvaluator;
using standard_robot_pp_ros2::YawAuthority;
using std::chrono::milliseconds;

namespace
{

GimbalYawStatusConfig realityConfig()
{
  GimbalYawStatusConfig config;
  config.joint_state_timeout = milliseconds(100);
  config.tf_timeout = milliseconds(200);
  config.lock_rate_threshold = 0.05;
  config.lock_dwell = milliseconds(100);
  config.allow_body_yaw_follow = false;
  return config;
}

// 送入若干帧静止的健康反馈，返回最后一帧时刻。
std::chrono::steady_clock::time_point feedStill(
  GimbalYawStatusEvaluator & evaluator, std::chrono::steady_clock::time_point start, int frames = 8,
  double yaw = 0.3)
{
  auto stamp = start;
  for (int i = 0; i < frames; ++i) {
    evaluator.onFeedback(yaw, 0.1, true, stamp);
    stamp += milliseconds(20);
  }
  return stamp - milliseconds(20);
}

}  // namespace

TEST(GimbalYawStatusEvaluator, NoFeedbackMeansUnhealthyAndSafeStop)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto output = evaluator.evaluate(std::chrono::steady_clock::now());
  EXPECT_FALSE(output.tf_healthy);
  EXPECT_FALSE(output.locked);
  EXPECT_EQ(output.yaw_authority, YawAuthority::HOLD_SAFE_STOP);
  EXPECT_EQ(output.request_sequence, 0U);
  EXPECT_EQ(output.reason, GimbalAckReason::FEEDBACK_STALE);
}

TEST(GimbalYawStatusEvaluator, TfFailureKeepsHealthyFalse)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto t0 = std::chrono::steady_clock::now();
  // TF 查询失败：即使串口反馈新鲜，也不得写成 tf_healthy=true。
  evaluator.onFeedback(0.3, 0.1, false, t0);
  ASSERT_TRUE(evaluator.onRequest(1, YawAuthority::GIMBAL_COMPENSATED, false));
  const auto output = evaluator.evaluate(t0 + milliseconds(10));
  EXPECT_FALSE(output.tf_healthy);
  EXPECT_EQ(output.yaw_authority, YawAuthority::HOLD_SAFE_STOP);
  EXPECT_EQ(output.reason, GimbalAckReason::TF_UNHEALTHY);
}

TEST(GimbalYawStatusEvaluator, HealthyFeedbackWithoutRequestDoesNotAck)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto t = feedStill(evaluator, std::chrono::steady_clock::now());
  const auto output = evaluator.evaluate(t + milliseconds(10));
  EXPECT_TRUE(output.tf_healthy);
  EXPECT_EQ(output.request_sequence, 0U);
  EXPECT_EQ(output.yaw_authority, YawAuthority::HOLD_SAFE_STOP);
  EXPECT_EQ(output.reason, GimbalAckReason::NO_REQUEST);
}

TEST(GimbalYawStatusEvaluator, StaleFeedbackRevokesAck)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto t = feedStill(evaluator, std::chrono::steady_clock::now());
  ASSERT_TRUE(evaluator.onRequest(3, YawAuthority::GIMBAL_COMPENSATED, false));
  const auto granted = evaluator.evaluate(t + milliseconds(10));
  ASSERT_EQ(granted.reason, GimbalAckReason::ACK_GRANTED);

  // 超过 T_joint = 100 ms 之后必须撤回，且早于上游 500 ms 租约。
  const auto stale = evaluator.evaluate(t + milliseconds(101));
  EXPECT_FALSE(stale.tf_healthy);
  EXPECT_FALSE(stale.locked);
  EXPECT_EQ(stale.yaw_authority, YawAuthority::HOLD_SAFE_STOP);
  EXPECT_EQ(stale.reason, GimbalAckReason::FEEDBACK_STALE);
}

TEST(GimbalYawStatusEvaluator, GimbalCompensatedAckGranted)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto t = feedStill(evaluator, std::chrono::steady_clock::now());
  ASSERT_TRUE(evaluator.onRequest(7, YawAuthority::GIMBAL_COMPENSATED, true));
  const auto output = evaluator.evaluate(t + milliseconds(10));
  EXPECT_TRUE(output.tf_healthy);
  EXPECT_TRUE(output.locked);
  EXPECT_EQ(output.request_sequence, 7U);
  EXPECT_EQ(output.yaw_authority, YawAuthority::GIMBAL_COMPENSATED);
  EXPECT_EQ(output.reason, GimbalAckReason::ACK_GRANTED);
  // fake_yaw 取第一帧健康实测 yaw，作为 $\psi_0$ 参考零位。
  EXPECT_DOUBLE_EQ(output.fake_yaw, 0.3);
  EXPECT_DOUBLE_EQ(output.gimbal_yaw, 0.3);
  EXPECT_DOUBLE_EQ(output.body_yaw, 0.1);
}

TEST(GimbalYawStatusEvaluator, BodyYawFollowIsStructurallyRefused)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto t = feedStill(evaluator, std::chrono::steady_clock::now());
  ASSERT_TRUE(evaluator.onRequest(2, YawAuthority::BODY_YAW_FOLLOW, false));
  const auto output = evaluator.evaluate(t + milliseconds(10));
  // 实车无车体 yaw 交接机构：即使链路健康也只回报 HOLD_SAFE_STOP。
  EXPECT_TRUE(output.tf_healthy);
  EXPECT_EQ(output.yaw_authority, YawAuthority::HOLD_SAFE_STOP);
  EXPECT_EQ(output.reason, GimbalAckReason::BODY_YAW_FOLLOW_FORBIDDEN);
}

TEST(GimbalYawStatusEvaluator, MovingGimbalIsNotLocked)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  auto stamp = std::chrono::steady_clock::now();
  // 20 ms 每帧转 0.02 rad，即 1.0 rad/s，远超 0.05 rad/s 阈值。
  double yaw = 0.0;
  for (int i = 0; i < 8; ++i) {
    evaluator.onFeedback(yaw, 0.0, true, stamp);
    yaw += 0.02;
    stamp += milliseconds(20);
  }
  ASSERT_TRUE(evaluator.onRequest(4, YawAuthority::GIMBAL_COMPENSATED, true));
  const auto output = evaluator.evaluate(stamp - milliseconds(10));
  EXPECT_TRUE(output.tf_healthy);
  EXPECT_FALSE(output.locked);
  EXPECT_EQ(output.yaw_authority, YawAuthority::HOLD_SAFE_STOP);
  EXPECT_EQ(output.reason, GimbalAckReason::NOT_LOCKED);
}

TEST(GimbalYawStatusEvaluator, LockNeedsDwellNotJustOneStillFrame)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  auto stamp = std::chrono::steady_clock::now();
  evaluator.onFeedback(0.0, 0.0, true, stamp);
  stamp += milliseconds(20);
  evaluator.onFeedback(0.4, 0.0, true, stamp);  // 转动中
  stamp += milliseconds(20);
  evaluator.onFeedback(0.4, 0.0, true, stamp);  // 刚停住
  ASSERT_TRUE(evaluator.onRequest(1, YawAuthority::GIMBAL_COMPENSATED, true));
  // dwell 尚未满足 100 ms。
  const auto early = evaluator.evaluate(stamp + milliseconds(50));
  EXPECT_FALSE(early.locked);
  EXPECT_EQ(early.reason, GimbalAckReason::NOT_LOCKED);
  // 继续静止，dwell 满足后才 locked。
  for (int i = 0; i < 6; ++i) {
    stamp += milliseconds(20);
    evaluator.onFeedback(0.4, 0.0, true, stamp);
  }
  const auto late = evaluator.evaluate(stamp + milliseconds(10));
  EXPECT_TRUE(late.locked);
  EXPECT_EQ(late.reason, GimbalAckReason::ACK_GRANTED);
}

TEST(GimbalYawStatusEvaluator, NonMonotonicRequestRejected)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto t = feedStill(evaluator, std::chrono::steady_clock::now());
  ASSERT_TRUE(evaluator.onRequest(9, YawAuthority::GIMBAL_COMPENSATED, false));
  EXPECT_FALSE(evaluator.onRequest(9, YawAuthority::BODY_YAW_FOLLOW, false));
  EXPECT_FALSE(evaluator.onRequest(8, YawAuthority::BODY_YAW_FOLLOW, false));
  const auto output = evaluator.evaluate(t + milliseconds(10));
  // 被拒的请求不得改变已回报的序号与授权。
  EXPECT_EQ(output.request_sequence, 9U);
  EXPECT_EQ(output.yaw_authority, YawAuthority::GIMBAL_COMPENSATED);
}

TEST(GimbalYawStatusEvaluator, RequestSequenceIsEchoedExactly)
{
  GimbalYawStatusEvaluator evaluator(realityConfig());
  const auto t = feedStill(evaluator, std::chrono::steady_clock::now());
  ASSERT_TRUE(evaluator.onRequest(11, YawAuthority::GIMBAL_COMPENSATED, false));
  // 上游按精确相等比较 request_sequence，因此不能回报别的值。
  EXPECT_EQ(evaluator.evaluate(t + milliseconds(5)).request_sequence, 11U);
  ASSERT_TRUE(evaluator.onRequest(12, YawAuthority::HOLD_SAFE_STOP, false));
  EXPECT_EQ(evaluator.evaluate(t + milliseconds(5)).request_sequence, 12U);
}
