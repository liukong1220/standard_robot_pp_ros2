// Copyright 2026
//
// 见 `include/standard_robot_pp_ros2/gimbal_yaw_status_bridge.hpp`。

#include "standard_robot_pp_ros2/gimbal_yaw_status_bridge.hpp"

#include <algorithm>
#include <cinttypes>
#include <memory>
#include <string>

#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"

namespace standard_robot_pp_ros2
{

using GimbalYawStatusMsg = ats_navigation_interfaces::msg::GimbalYawStatus;
using YawAuthorityRequestMsg = ats_navigation_interfaces::msg::YawAuthorityRequest;

GimbalYawStatusBridgeNode::GimbalYawStatusBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gimbal_yaw_status_bridge", options)
{
  joint_state_topic_ =
    declare_parameter<std::string>("joint_state_topic", "serial/gimbal_joint_state");
  request_topic_ =
    declare_parameter<std::string>("yaw_authority_request_topic", "/gimbal/yaw_authority_request");
  status_topic_ = declare_parameter<std::string>("gimbal_status_topic", "/gimbal/yaw_status");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  gimbal_yaw_frame_ = declare_parameter<std::string>("gimbal_yaw_frame", "gimbal_yaw_odom");
  body_frame_ = declare_parameter<std::string>("body_frame", "base_footprint");
  big_yaw_joint_name_ =
    declare_parameter<std::string>("big_yaw_joint_name", "gimbal_yaw_odom_joint");

  GimbalYawStatusConfig config;
  config.joint_state_timeout = std::chrono::milliseconds(
    std::max<int64_t>(1, declare_parameter<int>("joint_state_timeout_ms", 100)));
  config.tf_timeout =
    std::chrono::milliseconds(std::max<int64_t>(1, declare_parameter<int>("tf_timeout_ms", 200)));
  config.lock_rate_threshold =
    std::max(1e-4, declare_parameter<double>("lock_rate_threshold", 0.05));
  config.lock_dwell =
    std::chrono::milliseconds(std::max<int64_t>(0, declare_parameter<int>("lock_dwell_ms", 100)));
  // 实车恒为 false：没有车体 yaw 交接机构，BODY_YAW_FOLLOW 必须被拒绝。
  config.allow_body_yaw_follow = declare_parameter<bool>("allow_body_yaw_follow", false);

  const double publish_rate = std::max(1.0, declare_parameter<double>("publish_rate_hz", 50.0));
  const double publish_period_ms = 1000.0 / publish_rate;
  if (publish_period_ms >= static_cast<double>(config.joint_state_timeout.count())) {
    RCLCPP_ERROR(
      get_logger(),
      "publish_rate_hz=%.1f 的周期 %.1f ms 不小于 joint_state_timeout_ms=%ld："
      "本节点会在自己都还没发一帧时就判定反馈过期。已按 T_pub < T_joint 收紧发布周期。",
      publish_rate, publish_period_ms, static_cast<int64_t>(config.joint_state_timeout.count()));
  }
  if (config.joint_state_timeout > config.tf_timeout) {
    RCLCPP_ERROR(
      get_logger(),
      "joint_state_timeout_ms=%ld 超过 tf_timeout_ms=%ld，违反 T_joint <= T_tf。已取较小值。",
      static_cast<int64_t>(config.joint_state_timeout.count()),
      static_cast<int64_t>(config.tf_timeout.count()));
    config.tf_timeout = config.joint_state_timeout;
  }
  if (config.allow_body_yaw_follow) {
    RCLCPP_ERROR(
      get_logger(),
      "allow_body_yaw_follow=true：实车没有车体 yaw 交接机构，该配置只允许出现在"
      "明确标注的仿真或对照 profile。实车上必须为 false。");
  }
  evaluator_.setConfig(config);

  RCLCPP_INFO(
    get_logger(),
    "云台 ack 判据：T_joint=%ld ms，T_tf=%ld ms，T_pub=%.1f ms，"
    "lock_rate<=%.3f rad/s 持续 %ld ms；BODY_YAW_FOLLOW=%s",
    static_cast<int64_t>(config.joint_state_timeout.count()),
    static_cast<int64_t>(config.tf_timeout.count()), publish_period_ms, config.lock_rate_threshold,
    static_cast<int64_t>(config.lock_dwell.count()),
    config.allow_body_yaw_follow ? "允许（非实车）" : "结构性拒绝");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  // 与 Goal Manager、MPC 的订阅 QoS 一致。
  status_pub_ = create_publisher<GimbalYawStatusMsg>(
    status_topic_, rclcpp::QoS(1).reliable().transient_local());

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, rclcpp::SensorDataQoS(),
    std::bind(&GimbalYawStatusBridgeNode::jointStateCallback, this, std::placeholders::_1));

  request_sub_ = create_subscription<YawAuthorityRequestMsg>(
    request_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &GimbalYawStatusBridgeNode::yawAuthorityRequestCallback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::microseconds(static_cast<int64_t>(1e6 / std::min(publish_rate, 1000.0))),
    std::bind(&GimbalYawStatusBridgeNode::publishStatus, this));
}

bool GimbalYawStatusBridgeNode::lookupYaw(const std::string & frame, double & yaw) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(odom_frame_, frame, tf2::TimePointZero);
    yaw = tf2::getYaw(transform.transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void GimbalYawStatusBridgeNode::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 关节反馈只用于确认串口链路新鲜；yaw 数值仍取实测 TF，避免出现第二个
  // yaw 口径（关节角与 TF 不一致时必须暴露成 TF 不健康，而不是选一个用）。
  bool has_joint = false;
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (msg->name[i] == big_yaw_joint_name_) {
      has_joint = true;
      break;
    }
  }
  if (!has_joint) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s 中没有关节 %s：不能作为云台 yaw 反馈。",
      joint_state_topic_.c_str(), big_yaw_joint_name_.c_str());
    return;
  }

  double gimbal_yaw = 0.0;
  double body_yaw = 0.0;
  const bool gimbal_ok = lookupYaw(gimbal_yaw_frame_, gimbal_yaw);
  const bool body_ok = lookupYaw(body_frame_, body_yaw);
  if (!gimbal_ok || !body_ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TF 查询失败：%s->%s=%s，%s->%s=%s。tf_healthy 置 false，不伪造 ack。", odom_frame_.c_str(),
      gimbal_yaw_frame_.c_str(), gimbal_ok ? "ok" : "fail", odom_frame_.c_str(),
      body_frame_.c_str(), body_ok ? "ok" : "fail");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  evaluator_.onFeedback(
    gimbal_yaw, body_yaw, gimbal_ok && body_ok, std::chrono::steady_clock::now());
}

void GimbalYawStatusBridgeNode::yawAuthorityRequestCallback(
  const YawAuthorityRequestMsg::SharedPtr msg)
{
  bool accepted = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    accepted = evaluator_.onRequest(
      msg->request_sequence, static_cast<YawAuthority>(msg->yaw_authority),
      msg->require_gimbal_lock);
  }
  if (!accepted) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝 YawAuthorityRequest：request_sequence=%" PRIu64 " 不是严格递增。",
      msg->request_sequence);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "收到授权请求：request_sequence=%" PRIu64 "，yaw_authority=%u，require_lock=%s",
    msg->request_sequence, static_cast<unsigned>(msg->yaw_authority),
    msg->require_gimbal_lock ? "true" : "false");
}

void GimbalYawStatusBridgeNode::publishStatus()
{
  GimbalYawStatusMsg message;
  GimbalAckReason reason = GimbalAckReason::NO_REQUEST;
  bool reason_changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto output = evaluator_.evaluate(std::chrono::steady_clock::now());
    message.header.stamp = now();
    message.header.frame_id = gimbal_yaw_frame_;
    message.sequence = ++status_sequence_;
    message.request_sequence = output.request_sequence;
    message.yaw_authority = static_cast<uint8_t>(output.yaw_authority);
    message.locked = output.locked;
    // 实测量，不是模拟器 ack。
    message.tf_healthy = output.tf_healthy;
    message.gimbal_yaw = output.gimbal_yaw;
    message.body_yaw = output.body_yaw;
    message.fake_yaw = output.fake_yaw;
    reason = output.reason;
    reason_changed = reason != last_reason_;
    last_reason_ = reason;
  }
  status_pub_->publish(message);
  // 只在原因码变化时打日志，避免 50 Hz 刷屏。
  if (reason_changed && reason != GimbalAckReason::ACK_GRANTED) {
    RCLCPP_WARN(
      get_logger(),
      "云台 ack 被拒，原因码 %u（1=无请求 2=反馈过期 3=TF 不健康 "
      "4=BODY_YAW_FOLLOW 禁止 5=未锁定）：回报 HOLD_SAFE_STOP。",
      static_cast<unsigned>(reason));
  }
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::GimbalYawStatusBridgeNode)
