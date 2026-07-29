 

#ifndef STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
#define STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ats_navigation_interfaces/msg/execution_command.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ats_rm_interfaces/msg/buff.hpp"
#include "ats_rm_interfaces/msg/event_data.hpp"
#include "ats_rm_interfaces/msg/game_robot_hp.hpp"
#include "ats_rm_interfaces/msg/game_status.hpp"
#include "ats_rm_interfaces/msg/ground_robot_position.hpp"
#include "ats_rm_interfaces/msg/rfid_status.hpp"
#include "ats_rm_interfaces/msg/robot_state_info.hpp"
#include "ats_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "serial_driver/serial_driver.hpp"
#include "standard_robot_pp_ros2/cmd_vel_authorization.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "standard_robot_pp_ros2/robot_info.hpp"
#include "std_msgs/msg/bool.hpp"

namespace standard_robot_pp_ros2
{
class StandardRobotPpRos2Node : public rclcpp::Node
{
public:
  explicit StandardRobotPpRos2Node(const rclcpp::NodeOptions & options);

  ~StandardRobotPpRos2Node() override;

private:
  // 串口链路健康标志。由 receive/send 线程置 false，由 serialPortProtect 置 true；
  // 多线程读写，必须是原子量，且每次翻转都要同步通知授权门。
  std::atomic<bool> is_usb_ok_{false};
  bool debug_;
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  bool record_rosbag_;
  bool set_detector_color_;
  bool publish_imu_as_gimbal_joint_state_;
  bool accept_legacy_two_axis_joint_state_;
  bool small_yaw_is_relative_;
  bool invert_small_yaw_;
  double small_yaw_offset_;
  // 姿态模式订阅话题，默认来自行为树发布的 decision/robot_mode。
  std::string robot_mode_topic_;
  bool enable_transient_zero_cmd_hold_ = true;
  int transient_zero_cmd_hold_timeout_ms_ = 50;
  double transient_zero_cmd_linear_epsilon_ = 1e-3;
  double transient_zero_cmd_angular_epsilon_ = 1e-3;
  int cmd_vel_watchdog_timeout_ms_ = 300;
  // Nav2-free 官方 profile 必须为 true：没有 ExecutionCommand 授权时出口恒零。
  bool require_execution_authorization_ = false;
  std::string execution_command_topic_;
  std::string emergency_stop_topic_;
  double execution_command_timeout_ = 0.5;
  // 出口授权与归零判据的唯一实现，见 cmd_vel_authorization.hpp。
  CmdVelAuthorizationGate cmd_vel_gate_;
  CmdVelGateReason last_gate_reason_ = CmdVelGateReason::NO_AUTHORIZATION;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  std::thread receive_thread_;
  std::thread send_thread_;
  std::thread serial_port_protect_thread_;

  // Publish
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::RobotStateInfo>::SharedPtr robot_state_info_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::EventData>::SharedPtr event_data_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::GameRobotHP>::SharedPtr all_robot_hp_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr robot_motion_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::GroundRobotPosition>::SharedPtr
    ground_robot_position_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<ats_rm_interfaces::msg::Buff>::SharedPtr buff_pub_;

  // Subscribe
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_flag_sub_;
  // 执行授权与急停。授权是唯一放行来源，单独的 emergency_stop=false 不恢复运动。
  rclcpp::Subscription<ats_navigation_interfaces::msg::ExecutionCommand>::SharedPtr
    execution_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_gimbal_joint_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_shoot_sub_;
  // 订阅上层姿态模式，并写入串口发送结构体中的 speed_vector.mode。
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr robot_mode_sub_;
  RobotModels robot_models_;
  std::unordered_map<std::string, rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr>
    debug_pub_map_;

  SendRobotCmdData send_robot_cmd_data_{};
  std::mutex send_cmd_mutex_;

  void getParams();
  void createPublisher();
  void createSubscription();
  void createNewDebugPublisher(const std::string & name);
  void receiveData();
  void sendData();
  void serialPortProtect();

  void publishDebugData(ReceiveDebugData & data);
  void publishImuData(ReceiveImuData & data);
  void publishRobotInfo(ReceiveRobotInfoData & data);
  void publishEventData(ReceiveEventData & data);
  void publishAllRobotHp(ReceiveAllRobotHpData & data);
  void publishGameStatus(ReceiveGameStatusData & data);
  void publishRobotMotion(ReceiveRobotMotionData & data);
  void publishGroundRobotPosition(ReceiveGroundRobotPosition & data);
  void publishRfidStatus(ReceiveRfidStatus & data);
  void publishRobotStatus(ReceiveRobotStatus & data);
  void publishJointState(ReceiveJointState & data);
  void publishLegacyJointState(ReceiveLegacyJointState & data);
  void publishBuff(ReceiveBuff & data);

  // 把授权门的判定结果写入发送结构体，包含 speed_vector.stop。
  void applyGateOutputLocked(const CmdVelGateOutput & output);
  // 链路状态翻转的唯一入口：同步 is_usb_ok_ 与授权门，断连即归零。
  void setSerialLinkState(bool up);
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & params);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void executionCommandCallback(
    const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr msg);
  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void cmdGimbalJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
  // 把行为树发来的 move/attack/defend 模式映射到下位机协议字段。
  void cmdRobotModeCallback(const example_interfaces::msg::UInt8::SharedPtr msg);
  void setParam(const rclcpp::Parameter & param);
  bool getDetectColor(uint8_t robot_id, uint8_t & color);
  bool callTriggerService(const std::string & service_name);

  // Param client to set detect_color
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;
  std::string detector_node_name_;

  uint8_t previous_game_progress_ = 0;
  int32_t last_logged_stage_remain_time_ = -1;
  int last_logged_current_hp_ = -1;
  int last_logged_maximum_hp_ = -1;
  int last_logged_projectile_allowance_17mm_ = -1;
  int last_logged_heat_ = -1;
  int last_logged_armor_id_ = -1;
  int last_logged_hp_reason_ = -1;
  bool last_logged_is_hp_deduced_ = false;
  uint8_t previous_robot_mode_cmd_ = 0;

  float last_hp_ = -1.0F;
  // rclcpp::Time stop_start_time_;
};
}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
