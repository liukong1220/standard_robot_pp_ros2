// Copyright 2026
//
// 实车 `GimbalYawStatus` 发布者（P6 零.3）。
//
// 反馈源是串口节点已经发布的 `serial/gimbal_joint_state`
// （`standard_robot_pp_ros2.cpp` 的 `publishJointState()`，关节名
// `gimbal_yaw_odom_joint` / `gimbal_yaw_joint` / `gimbal_pitch_joint`），
// 以及 `odom->gimbal_yaw_odom`、`odom->base_footprint` 两条实测 TF。
//
// 判据全部在 `gimbal_yaw_status_logic.hpp` 中，本文件只负责取数与发布。
// 本节点是实车上 `GimbalYawStatus` 的唯一发布者；MuJoCo 的模拟 ack 只在仿真
// profile 存在，两者不得同时运行。

#ifndef STANDARD_ROBOT_PP_ROS2__GIMBAL_YAW_STATUS_BRIDGE_HPP_
#define STANDARD_ROBOT_PP_ROS2__GIMBAL_YAW_STATUS_BRIDGE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "ats_navigation_interfaces/msg/gimbal_yaw_status.hpp"
#include "ats_navigation_interfaces/msg/yaw_authority_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "standard_robot_pp_ros2/gimbal_yaw_status_logic.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace standard_robot_pp_ros2
{

class GimbalYawStatusBridgeNode : public rclcpp::Node
{
public:
  explicit GimbalYawStatusBridgeNode(const rclcpp::NodeOptions & options);

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void yawAuthorityRequestCallback(
    const ats_navigation_interfaces::msg::YawAuthorityRequest::SharedPtr msg);
  void publishStatus();

  /// 查询 `odom->frame` 的 yaw。失败返回 false，绝不返回猜测值。
  bool lookupYaw(const std::string & frame, double & yaw) const;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::YawAuthorityRequest>::SharedPtr request_sub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::GimbalYawStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string joint_state_topic_;
  std::string request_topic_;
  std::string status_topic_;
  std::string odom_frame_;
  std::string gimbal_yaw_frame_;
  std::string body_frame_;
  std::string big_yaw_joint_name_;

  std::mutex mutex_;
  GimbalYawStatusEvaluator evaluator_;
  uint64_t status_sequence_ = 0;
  GimbalAckReason last_reason_ = GimbalAckReason::NO_REQUEST;
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__GIMBAL_YAW_STATUS_BRIDGE_HPP_
