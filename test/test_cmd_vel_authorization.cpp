// Copyright 2026
//
// 出口授权门的判据测试（P6 一.2/一.3）。覆盖：
//  - 授权归零立即生效，不被瞬时零保持吞掉
//  - 通信抖动的瞬时零才允许保持，且窗口上界受限
//  - 断连立即归零，重连不自动恢复旧授权
//  - 单独的 emergency_stop=false 不构成授权
//  - 序号回退与超龄授权样本被拒绝
//  - 看门狗超时归零且置 stop

#include <gtest/gtest.h>

#include <chrono>

#include "standard_robot_pp_ros2/cmd_vel_authorization.hpp"

using standard_robot_pp_ros2::CmdVelAuthorizationGate;
using standard_robot_pp_ros2::CmdVelGateConfig;
using standard_robot_pp_ros2::CmdVelGateOutput;
using standard_robot_pp_ros2::CmdVelGateReason;
using std::chrono::milliseconds;

namespace
{

CmdVelGateConfig realityConfig()
{
  CmdVelGateConfig config;
  config.require_execution_authorization = true;
  config.enable_transient_zero_cmd_hold = true;
  config.transient_zero_cmd_hold_timeout = milliseconds(50);
  config.cmd_vel_watchdog_timeout = milliseconds(300);
  config.execution_command_timeout = milliseconds(500);
  return config;
}

// 建立一次合法授权，返回授权时刻。
std::chrono::steady_clock::time_point authorize(
  CmdVelAuthorizationGate & gate, uint64_t sequence = 1)
{
  EXPECT_TRUE(gate.onExecutionCommand(true, sequence, 7, 11, milliseconds(0)));
  EXPECT_TRUE(gate.authorized());
  return std::chrono::steady_clock::now();
}

}  // namespace

TEST(CmdVelAuthorizationGate, LinkDownUntilPortOpens)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkDown();
  const auto now = std::chrono::steady_clock::now();
  const auto output = gate.onCmdVel(1.0, 0.5, 0.3, now);
  EXPECT_TRUE(output.isZero());
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(output.reason, CmdVelGateReason::LINK_DOWN);
}

TEST(CmdVelAuthorizationGate, NoAuthorizationKeepsExitZero)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto output = gate.onCmdVel(1.2, -0.4, 0.9, std::chrono::steady_clock::now());
  EXPECT_TRUE(output.isZero());
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(output.reason, CmdVelGateReason::NO_AUTHORIZATION);
}

TEST(CmdVelAuthorizationGate, AuthorizedCommandPassesThroughUnchanged)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto now = authorize(gate);
  const auto output = gate.onCmdVel(1.5, -1.5, 2.0, now);
  // 车体系数值与方向必须逐值不变，出口不做旋转也不做增益。
  EXPECT_DOUBLE_EQ(output.vx, 1.5);
  EXPECT_DOUBLE_EQ(output.vy, -1.5);
  EXPECT_DOUBLE_EQ(output.wz, 2.0);
  EXPECT_FALSE(output.stop);
  EXPECT_EQ(output.reason, CmdVelGateReason::FORWARD);
}

TEST(CmdVelAuthorizationGate, TransientZeroHoldsWithinWindow)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto t0 = authorize(gate);
  gate.onCmdVel(1.0, 0.0, 0.0, t0);
  const auto held = gate.onCmdVel(0.0, 0.0, 0.0, t0 + milliseconds(40));
  EXPECT_DOUBLE_EQ(held.vx, 1.0);
  EXPECT_FALSE(held.stop);
  EXPECT_EQ(held.reason, CmdVelGateReason::HOLD_TRANSIENT_ZERO);
}

TEST(CmdVelAuthorizationGate, TransientZeroStopsHoldingPastWindow)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto t0 = authorize(gate);
  gate.onCmdVel(1.0, 0.0, 0.0, t0);
  const auto released = gate.onCmdVel(0.0, 0.0, 0.0, t0 + milliseconds(51));
  EXPECT_TRUE(released.isZero());
  EXPECT_EQ(released.reason, CmdVelGateReason::FORWARD);
}

TEST(CmdVelAuthorizationGate, ExecutionStopBypassesTransientZeroHold)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto t0 = authorize(gate, 1);
  gate.onCmdVel(1.0, 0.0, 0.0, t0);
  // STOP 在保持窗口内到达：必须立即归零，而不是等窗口过期。
  ASSERT_TRUE(gate.onExecutionCommand(false, 2, 7, 11, milliseconds(0)));
  EXPECT_FALSE(gate.authorized());
  const auto output = gate.onCmdVel(0.0, 0.0, 0.0, t0 + milliseconds(10));
  EXPECT_TRUE(output.isZero());
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(output.reason, CmdVelGateReason::AUTHORIZED_ZERO);
}

TEST(CmdVelAuthorizationGate, EmergencyStopBypassesTransientZeroHold)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto t0 = authorize(gate);
  gate.onCmdVel(0.8, 0.0, 0.0, t0);
  gate.onEmergencyStop(true);
  const auto output = gate.onCmdVel(0.0, 0.0, 0.0, t0 + milliseconds(5));
  EXPECT_TRUE(output.isZero());
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(output.reason, CmdVelGateReason::AUTHORIZED_ZERO);
}

TEST(CmdVelAuthorizationGate, EmergencyStopFalseAloneDoesNotAuthorize)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  authorize(gate);
  gate.onEmergencyStop(true);
  gate.onEmergencyStop(false);
  EXPECT_FALSE(gate.authorized());
  const auto output = gate.onCmdVel(1.0, 0.0, 0.0, std::chrono::steady_clock::now());
  EXPECT_TRUE(output.isZero());
  EXPECT_EQ(output.reason, CmdVelGateReason::NO_AUTHORIZATION);
}

TEST(CmdVelAuthorizationGate, ReconnectDoesNotRestoreOldAuthorization)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto t0 = authorize(gate, 5);
  gate.onCmdVel(1.0, 0.0, 0.0, t0);

  gate.onSerialLinkDown();
  EXPECT_FALSE(gate.authorized());
  gate.onSerialLinkUp();
  // 链路恢复但授权未恢复。
  EXPECT_TRUE(gate.linkUp());
  EXPECT_FALSE(gate.authorized());
  const auto blocked = gate.onCmdVel(1.0, 0.0, 0.0, t0 + milliseconds(1000));
  EXPECT_TRUE(blocked.isZero());
  EXPECT_EQ(blocked.reason, CmdVelGateReason::NO_AUTHORIZATION);

  // 旧序号不能重新授权。
  EXPECT_FALSE(gate.onExecutionCommand(true, 5, 7, 11, milliseconds(0)));
  EXPECT_FALSE(gate.authorized());
  // 新一代（序号更大、epoch/generation 更新）才恢复授权。
  EXPECT_TRUE(gate.onExecutionCommand(true, 6, 8, 12, milliseconds(0)));
  EXPECT_TRUE(gate.authorized());
  EXPECT_EQ(gate.lastLocalizationEpoch(), 8U);
  EXPECT_EQ(gate.lastMapGeneration(), 12U);
  const auto allowed = gate.onCmdVel(1.0, 0.0, 0.0, t0 + milliseconds(1100));
  EXPECT_DOUBLE_EQ(allowed.vx, 1.0);
  EXPECT_EQ(allowed.reason, CmdVelGateReason::FORWARD);
}

TEST(CmdVelAuthorizationGate, AuthorizationDuringLinkDownIsNotHonored)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkDown();
  EXPECT_TRUE(gate.onExecutionCommand(true, 1, 7, 11, milliseconds(0)));
  // 样本被记录（序号推进），但断连期间不产生授权。
  EXPECT_FALSE(gate.authorized());
}

TEST(CmdVelAuthorizationGate, StaleAuthorizationSampleRejected)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  // transient_local 重投的旧样本：年龄超过 500 ms 租约必须被拒。
  EXPECT_FALSE(gate.onExecutionCommand(true, 1, 7, 11, milliseconds(501)));
  EXPECT_FALSE(gate.authorized());
}

TEST(CmdVelAuthorizationGate, WatchdogZeroesAndAssertsStop)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto t0 = authorize(gate);
  gate.onCmdVel(1.0, 0.0, 0.0, t0);

  CmdVelGateOutput output;
  EXPECT_FALSE(gate.tick(t0 + milliseconds(300), output));
  ASSERT_TRUE(gate.tick(t0 + milliseconds(301), output));
  EXPECT_TRUE(output.isZero());
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(output.reason, CmdVelGateReason::WATCHDOG_TIMEOUT);
}

TEST(CmdVelAuthorizationGate, TickZeroesWhenLinkDropsWithoutNewCmd)
{
  CmdVelAuthorizationGate gate(realityConfig());
  gate.onSerialLinkUp();
  const auto t0 = authorize(gate);
  gate.onCmdVel(1.0, 0.0, 0.0, t0);
  gate.onSerialLinkDown();

  CmdVelGateOutput output;
  ASSERT_TRUE(gate.tick(t0 + milliseconds(1), output));
  EXPECT_TRUE(output.isZero());
  EXPECT_TRUE(output.stop);
  EXPECT_EQ(output.reason, CmdVelGateReason::LINK_DOWN);
}

TEST(CmdVelAuthorizationGate, Nav2ComparisonProfileNeedsNoAuthorization)
{
  CmdVelGateConfig config = realityConfig();
  config.require_execution_authorization = false;
  CmdVelAuthorizationGate gate(config);
  gate.onSerialLinkUp();
  const auto output = gate.onCmdVel(0.6, 0.0, 0.2, std::chrono::steady_clock::now());
  EXPECT_DOUBLE_EQ(output.vx, 0.6);
  EXPECT_EQ(output.reason, CmdVelGateReason::FORWARD);
  // 但急停仍然立即生效。
  gate.onEmergencyStop(true);
  const auto stopped = gate.onCmdVel(0.6, 0.0, 0.2, std::chrono::steady_clock::now());
  EXPECT_TRUE(stopped.isZero());
  EXPECT_TRUE(stopped.stop);
}
