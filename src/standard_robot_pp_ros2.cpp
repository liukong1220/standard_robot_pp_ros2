 

#include "standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{

namespace
{
double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// 判零逻辑已上移到 `CmdVelAuthorizationGate`（cmd_vel_authorization.hpp），
// 以便与授权、断连、看门狗判据在同一处被单元测试覆盖。
// 原本地静态函数 isNearlyZeroTwist 因此成为未使用符号，本包 -Werror
// 会直接报错，故随本次重构一并移除，不是无关清理。

const char * gameProgressName(const uint8_t progress)
{
  switch (progress) {
    case ats_rm_interfaces::msg::GameStatus::NOT_START:
      return "NOT_START";
    case ats_rm_interfaces::msg::GameStatus::PREPARATION:
      return "PREPARATION";
    case ats_rm_interfaces::msg::GameStatus::SELF_CHECKING:
      return "SELF_CHECKING";
    case ats_rm_interfaces::msg::GameStatus::COUNT_DOWN:
      return "COUNT_DOWN";
    case ats_rm_interfaces::msg::GameStatus::RUNNING:
      return "RUNNING";
    case ats_rm_interfaces::msg::GameStatus::GAME_OVER:
      return "GAME_OVER";
    default:
      return "UNKNOWN";
  }
}

const char * hpDeductionReasonName(const uint8_t reason)
{
  switch (reason) {
    case ats_rm_interfaces::msg::RobotStatus::ARMOR_HIT:
      return "ARMOR_HIT";
    case ats_rm_interfaces::msg::RobotStatus::SYSTEM_OFFLINE:
      return "SYSTEM_OFFLINE";
    case ats_rm_interfaces::msg::RobotStatus::OVER_SHOOT_SPEED:
      return "OVER_SHOOT_SPEED";
    case ats_rm_interfaces::msg::RobotStatus::OVER_HEAT:
      return "OVER_HEAT";
    case ats_rm_interfaces::msg::RobotStatus::OVER_POWER:
      return "OVER_POWER";
    case ats_rm_interfaces::msg::RobotStatus::ARMOR_COLLISION:
      return "ARMOR_COLLISION";
    default:
      return "UNKNOWN";
  }
}

const char * robotModeName(const uint8_t mode)
{
  switch (mode) {
    case 1:
      return "attack";
    case 2:
      return "defend";
    case 3:
      return "move";
    default:
      return "unknown";
  }
}
}  // namespace

StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");

  getParams();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&StandardRobotPpRos2Node::onParametersSet, this, std::placeholders::_1));
  createPublisher();
  createSubscription();

  robot_models_.chassis = {
    {0, "无底盘"}, {1, "麦轮底盘"}, {2, "全向轮底盘"}, {3, "舵轮底盘"}, {4, "平衡底盘"}};
  robot_models_.gimbal = {{0, "无云台"}, {1, "yaw_pitch直连云台"}};
  robot_models_.shoot = {{0, "无发射机构"}, {1, "摩擦轮+拨弹盘"}, {2, "气动+拨弹盘"}};
  robot_models_.arm = {{0, "无机械臂"}, {1, "mini机械臂"}};
  robot_models_.custom_controller = {{0, "无自定义控制器"}, {1, "mini自定义控制器"}};

  serial_port_protect_thread_ = std::thread(&StandardRobotPpRos2Node::serialPortProtect, this);
  receive_thread_ = std::thread(&StandardRobotPpRos2Node::receiveData, this);
  send_thread_ = std::thread(&StandardRobotPpRos2Node::sendData, this);
}

StandardRobotPpRos2Node::~StandardRobotPpRos2Node()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_port_protect_thread_.joinable()) {
    serial_port_protect_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void StandardRobotPpRos2Node::createPublisher()
{
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);
  robot_state_info_pub_ =
    this->create_publisher<ats_rm_interfaces::msg::RobotStateInfo>("serial/robot_state_info", 10);
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
  robot_motion_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("serial/robot_motion", 10);
  event_data_pub_ =
    this->create_publisher<ats_rm_interfaces::msg::EventData>("referee/event_data", 10);
  all_robot_hp_pub_ =
    this->create_publisher<ats_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", 10);
  game_status_pub_ =
    this->create_publisher<ats_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  ground_robot_position_pub_ = this->create_publisher<ats_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", 10);
  rfid_status_pub_ =
    this->create_publisher<ats_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<ats_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  buff_pub_ = this->create_publisher<ats_rm_interfaces::msg::Buff>("referee/buff", 10);
}

void StandardRobotPpRos2Node::createNewDebugPublisher(const std::string & name)
{
  RCLCPP_INFO(get_logger(), "Create new debug publisher: %s", name.c_str());
  std::string topic_name = "serial/debug/" + name;
  auto debug_pub = this->create_publisher<example_interfaces::msg::Float64>(topic_name, 10);
  debug_pub_map_.insert(std::make_pair(name, debug_pub));
}

void StandardRobotPpRos2Node::createSubscription()
{
  // 旧的 stop_flag 只是把协议位写成 true/false，既不归零速度也不撤销授权。
  // 现在统一走授权门：true 等价于授权归零，false 只清标志、不恢复运动。
  stop_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "stop_flag", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) { emergencyStopCallback(msg); });

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&StandardRobotPpRos2Node::cmdVelCallback, this, std::placeholders::_1));

  // 执行授权。QoS 必须与 Goal Manager 的发布端一致
  // （`rclcpp::QoS(1).reliable().transient_local()`），否则订阅不匹配、
  // 本节点会永远停在 NO_AUTHORIZATION 而表现为「底盘完全不动」。
  if (!execution_command_topic_.empty()) {
    execution_command_sub_ =
      this->create_subscription<ats_navigation_interfaces::msg::ExecutionCommand>(
        execution_command_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&StandardRobotPpRos2Node::executionCommandCallback, this, std::placeholders::_1));
  } else if (require_execution_authorization_) {
    throw std::invalid_argument(
      "require_execution_authorization=true 时 execution_command_topic 不能为空");
  }

  // 急停只能收紧、不能放行：true 立即归零，false 不恢复授权。
  if (!emergency_stop_topic_.empty()) {
    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&StandardRobotPpRos2Node::emergencyStopCallback, this, std::placeholders::_1));
  }

  cmd_gimbal_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "cmd_gimbal_joint", 10,
    std::bind(&StandardRobotPpRos2Node::cmdGimbalJointCallback, this, std::placeholders::_1));

  cmd_shoot_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
    "cmd_shoot", 10,
    std::bind(&StandardRobotPpRos2Node::cmdShootCallback, this, std::placeholders::_1));

  robot_mode_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
    robot_mode_topic_, 10,
    std::bind(&StandardRobotPpRos2Node::cmdRobotModeCallback, this, std::placeholders::_1));
}

void StandardRobotPpRos2Node::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

  record_rosbag_ = declare_parameter("record_rosbag", false);
  set_detector_color_ = declare_parameter("set_detector_color", false);
  debug_ = declare_parameter("debug", false);
  publish_imu_as_gimbal_joint_state_ = declare_parameter("publish_imu_as_gimbal_joint_state", false);
  accept_legacy_two_axis_joint_state_ =
    declare_parameter("accept_legacy_two_axis_joint_state", false);
  small_yaw_is_relative_ = declare_parameter("small_yaw_is_relative", true);
  invert_small_yaw_ = declare_parameter("invert_small_yaw", false);
  small_yaw_offset_ = declare_parameter("small_yaw_offset", 0.0);
  enable_transient_zero_cmd_hold_ = declare_parameter("enable_transient_zero_cmd_hold", true);
  // 代码默认必须与 `config/standard_robot_pp_ros2.yaml` 一致。此前代码默认 150 ms、
  // 配置 50 ms，任何忘记传参数文件的启动方式都会静默把残余运动窗口放大到 3 倍。
  transient_zero_cmd_hold_timeout_ms_ = declare_parameter("transient_zero_cmd_hold_timeout_ms", 50);
  transient_zero_cmd_linear_epsilon_ = declare_parameter("transient_zero_cmd_linear_epsilon", 1e-3);
  transient_zero_cmd_angular_epsilon_ =
    declare_parameter("transient_zero_cmd_angular_epsilon", 1e-3);
  cmd_vel_watchdog_timeout_ms_ = declare_parameter("cmd_vel_watchdog_timeout_ms", 300);
  require_execution_authorization_ = declare_parameter("require_execution_authorization", false);
  execution_command_topic_ =
    declare_parameter("execution_command_topic", std::string("/planner/execution_command"));
  emergency_stop_topic_ =
    declare_parameter("emergency_stop_topic", std::string("/planner/emergency_stop"));
  execution_command_timeout_ = declare_parameter("execution_command_timeout", 0.5);

  // 时序关系必须成立，否则底盘这一级会比上游租约更宽松，
  // 「上游超时 -> 全链归零」的上界推导在出口失效。
  if (cmd_vel_watchdog_timeout_ms_ > static_cast<int>(execution_command_timeout_ * 1000.0)) {
    RCLCPP_ERROR(
      get_logger(),
      "cmd_vel_watchdog_timeout_ms=%d 超过授权租约 %.0f ms：底盘看门狗必须不松于上游租约。"
      "已按租约收紧看门狗窗口。",
      cmd_vel_watchdog_timeout_ms_, execution_command_timeout_ * 1000.0);
    cmd_vel_watchdog_timeout_ms_ = static_cast<int>(execution_command_timeout_ * 1000.0);
  }
  if (transient_zero_cmd_hold_timeout_ms_ >= cmd_vel_watchdog_timeout_ms_) {
    RCLCPP_ERROR(
      get_logger(),
      "transient_zero_cmd_hold_timeout_ms=%d 不小于看门狗 %d ms：瞬时零保持会覆盖刹停判据。"
      "已关闭瞬时零保持。",
      transient_zero_cmd_hold_timeout_ms_, cmd_vel_watchdog_timeout_ms_);
    enable_transient_zero_cmd_hold_ = false;
  }

  CmdVelGateConfig gate_config;
  gate_config.require_execution_authorization = require_execution_authorization_;
  gate_config.enable_transient_zero_cmd_hold = enable_transient_zero_cmd_hold_;
  gate_config.transient_zero_cmd_hold_timeout =
    std::chrono::milliseconds(transient_zero_cmd_hold_timeout_ms_);
  gate_config.cmd_vel_watchdog_timeout = std::chrono::milliseconds(cmd_vel_watchdog_timeout_ms_);
  gate_config.execution_command_timeout =
    std::chrono::milliseconds(static_cast<int>(execution_command_timeout_ * 1000.0));
  gate_config.linear_epsilon = transient_zero_cmd_linear_epsilon_;
  gate_config.angular_epsilon = transient_zero_cmd_angular_epsilon_;
  cmd_vel_gate_.setConfig(gate_config);
  // 串口尚未打开前链路视为断开：出口在授权门里恒为零速度 + stop=true。
  cmd_vel_gate_.onSerialLinkDown();
  RCLCPP_INFO(
    get_logger(),
    "出口时序：T_hold=%d ms，T_wd=%d ms，T_lease=%.0f ms，require_execution_authorization=%s",
    transient_zero_cmd_hold_timeout_ms_, cmd_vel_watchdog_timeout_ms_,
    execution_command_timeout_ * 1000.0, require_execution_authorization_ ? "true" : "false");
  // 上层行为树通过该话题下发姿态模式，默认值与 ats_sentry_behavior 保持一致。
  robot_mode_topic_ = declare_parameter("robot_mode_topic", std::string("decision/robot_mode"));
}

/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  // 断连检测、重连、重连期间确定性归零，以及「重连成功不自动恢复旧授权」。
  //
  // 安全契约：
  //  1. 打开失败或任何一侧线程报错都会把链路标成 down，授权门立即失效并输出
  //     零速度 + `speed_vector.stop = true`；本轮不再发帧（`sendData` 的
  //     `!is_usb_ok_` 分支）。
  //  2. 重连成功只恢复链路，不恢复授权。恢复运动必须等 Goal Manager 发来一条
  //     `command_sequence` 严格更大且时间戳新鲜的 `MODE_EXECUTE`，即经过新的
  //     epoch/generation/序号重新授权。
  //  3. 端口存在性用 `is_open()` 加一次 0 字节探测判断，避免设备节点已消失
  //     （USB 拔出）而 `is_open()` 仍为真时把链路误判成健康。

  try {
    serial_driver_->init_port(device_name_, *device_config_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "init_port 失败：%s", ex.what());
  }

  try {
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
    }
    if (serial_driver_->port()->is_open()) {
      RCLCPP_INFO(get_logger(), "串口已打开：%s", device_name_.c_str());
      setSerialLinkState(true);
    } else {
      setSerialLinkState(false);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "打开串口失败：%s", ex.what());
    // 打开失败绝不能置 true。此前这里在 try/catch 之后无条件
    // `is_usb_ok_ = true;`，会让收发线程对着一个没打开的端口反复抛异常，
    // 同时把「链路健康」错误地报告给出口逻辑。
    setSerialLinkState(false);
  }

  int reconnect_attempts = 0;

  while (rclcpp::ok()) {
    if (is_usb_ok_.load()) {
      // 链路自认健康时仍要探测设备是否还在：USB 被拔出后 is_open() 可能仍为真。
      bool alive = false;
      try {
        alive = serial_driver_->port()->is_open() && std::filesystem::exists(device_name_);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "串口健康探测异常：%s", ex.what());
        alive = false;
      }
      if (!alive) {
        RCLCPP_ERROR(
          get_logger(), "检测到串口断连（%s）：立即归零并撤销执行授权。", device_name_.c_str());
        setSerialLinkState(false);
      }
    }

    if (!is_usb_ok_.load()) {
      try {
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        if (serial_driver_->port()->is_open()) {
          reconnect_attempts = 0;
          // 重连成功：只恢复链路。授权仍然是 false，出口继续零速度，
          // 直到新的 ExecutionCommand(EXECUTE) 带着更大的序号到达。
          setSerialLinkState(true);
          RCLCPP_WARN(
            get_logger(),
            "串口重连成功：链路已恢复，但执行授权未恢复。"
            "必须由新的 ExecutionCommand(EXECUTE) 重新授权后才会输出非零速度。");
        } else {
          setSerialLinkState(false);
        }
      } catch (const std::exception & ex) {
        setSerialLinkState(false);
        RCLCPP_ERROR(get_logger(), "串口重连失败（第 %d 次）：%s", ++reconnect_attempts, ex.what());
      }
    }

    // thread sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }
}

void StandardRobotPpRos2Node::setSerialLinkState(bool up)
{
  const bool was_up = is_usb_ok_.exchange(up);
  std::lock_guard<std::mutex> lock(send_cmd_mutex_);
  if (up) {
    cmd_vel_gate_.onSerialLinkUp();
  } else {
    cmd_vel_gate_.onSerialLinkDown();
    // 断连瞬间就把发送结构体钉成零速度 + stop，使得链路一旦恢复、
    // 第一帧也不可能是旧的非零命令。
    CmdVelGateOutput zero;
    zero.stop = true;
    zero.reason = CmdVelGateReason::LINK_DOWN;
    applyGateOutputLocked(zero);
  }
  if (was_up != up) {
    RCLCPP_WARN(get_logger(), "串口链路状态切换：%s", up ? "UP" : "DOWN");
  }
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  std::vector<uint8_t> sof(1);
  std::vector<uint8_t> receive_data;

  int sof_count = 0;
  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "receive: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      serial_driver_->port()->receive(sof);

      if (sof[0] != SOF_RECEIVE) {
        sof_count++;
        continue;
      }

      // Reset sof_count when SOF_RECEIVE is found
      sof_count = 0;

      // sof[0] == SOF_RECEIVE 后读取剩余 header_frame 内容
      std::vector<uint8_t> header_frame_buf(3);  // sof 在读取完数据后添加

      serial_driver_->port()->receive(header_frame_buf);  // 读取除 sof 外剩下的数据
      header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加 sof
      HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);

      // HeaderFrame CRC8 check
      bool crc8_ok = crc8::verify_CRC8_check_sum(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
      if (!crc8_ok) {
        RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
        continue;
      }

      // crc8_ok 校验正确后读取数据段
      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf(header_frame.len + 2);  // len + crc
      int received_len = serial_driver_->port()->receive(data_buf);
      int received_len_sum = received_len;
      // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
      // 计算剩余未读取的数据长度
      int remain_len = header_frame.len + 2 - received_len;
      while (remain_len > 0) {  // 读取剩余未读取的数据
        std::vector<uint8_t> remain_buf(remain_len);
        received_len = serial_driver_->port()->receive(remain_buf);
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        received_len_sum += received_len;
        remain_len -= received_len;
      }

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包
      data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

      if (!debug_ && header_frame.id == ID_DEBUG) {
        continue;
      }

      // 整包数据校验
      bool crc16_ok = crc16::verify_CRC16_check_sum(data_buf);
      if (!crc16_ok) {
        RCLCPP_ERROR(get_logger(), "Data segment CRC16 error!");
        continue;
      }

      // crc16_ok 校验正确后根据 header_frame.id 解析数据
      switch (header_frame.id) {
        case ID_DEBUG: {
          ReceiveDebugData debug_data = fromVector<ReceiveDebugData>(data_buf);
          publishDebugData(debug_data);
        } break;
        case ID_IMU: {
          ReceiveImuData imu_data = fromVector<ReceiveImuData>(data_buf);
          publishImuData(imu_data);
        } break;
        case ID_ROBOT_STATE_INFO: {
          ReceiveRobotInfoData robot_info_data = fromVector<ReceiveRobotInfoData>(data_buf);
          publishRobotInfo(robot_info_data);
        } break;
        case ID_EVENT_DATA: {
          ReceiveEventData event_data = fromVector<ReceiveEventData>(data_buf);
          publishEventData(event_data);
        } break;
        case ID_PID_DEBUG: {
        } break;
        case ID_ALL_ROBOT_HP: {
          ReceiveAllRobotHpData all_robot_hp_data = fromVector<ReceiveAllRobotHpData>(data_buf);
          publishAllRobotHp(all_robot_hp_data);
        } break;
        case ID_GAME_STATUS: {
          ReceiveGameStatusData game_status_data = fromVector<ReceiveGameStatusData>(data_buf);
          publishGameStatus(game_status_data);
        } break;
        case ID_ROBOT_MOTION: {
          ReceiveRobotMotionData robot_motion_data = fromVector<ReceiveRobotMotionData>(data_buf);
          publishRobotMotion(robot_motion_data);
        } break;
        case ID_GROUND_ROBOT_POSITION: {
          ReceiveGroundRobotPosition ground_robot_position_data =
            fromVector<ReceiveGroundRobotPosition>(data_buf);
          publishGroundRobotPosition(ground_robot_position_data);
        } break;
        case ID_RFID_STATUS: {
          ReceiveRfidStatus rfid_status_data = fromVector<ReceiveRfidStatus>(data_buf);
          publishRfidStatus(rfid_status_data);
        } break;
        case ID_ROBOT_STATUS: {
          ReceiveRobotStatus robot_status_data = fromVector<ReceiveRobotStatus>(data_buf);
          publishRobotStatus(robot_status_data);
        } break;
        case ID_JOINT_STATE: {
          if (header_frame.len == sizeof(ReceiveJointState) - sizeof(HeaderFrame) - 2U) {
            ReceiveJointState joint_state_data = fromVector<ReceiveJointState>(data_buf);
            publishJointState(joint_state_data);
          } else if (
            accept_legacy_two_axis_joint_state_ &&
            header_frame.len == sizeof(ReceiveLegacyJointState) - sizeof(HeaderFrame) - 2U)
          {
            ReceiveLegacyJointState joint_state_data = fromVector<ReceiveLegacyJointState>(data_buf);
            publishLegacyJointState(joint_state_data);
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Ignore ID_JOINT_STATE with unsupported payload length %u; expected %zu bytes for three-axis gimbal state",
              header_frame.len, sizeof(ReceiveJointState) - sizeof(HeaderFrame) - 2U);
          }
        } break;
        case ID_BUFF: {
          ReceiveBuff buff = fromVector<ReceiveBuff>(data_buf);
          publishBuff(buff);
        } break;
        default: {
          RCLCPP_WARN(get_logger(), "Invalid id: %d", header_frame.id);
        } break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      // 收侧异常也是链路故障：走统一入口，撤销授权并立即归零。
      setSerialLinkState(false);
    }
  }
}

void StandardRobotPpRos2Node::publishDebugData(ReceiveDebugData & received_debug_data)
{
  static rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr debug_pub;
  for (auto & package : received_debug_data.packages) {
    // Create a vector to hold the non-zero data
    std::vector<uint8_t> non_zero_data;
    for (unsigned char name : package.name) {
      if (name != 0) {
        non_zero_data.push_back(name);
      } else {
        break;
      }
    }
    // Convert the non-zero data to a string
    std::string name(non_zero_data.begin(), non_zero_data.end());

    if (name.empty()) {
      continue;
    }

    if (debug_pub_map_.find(name) == debug_pub_map_.end()) {
      createNewDebugPublisher(name);
    }
    debug_pub = debug_pub_map_.at(name);

    example_interfaces::msg::Float64 msg;
    msg.data = package.data;
    debug_pub->publish(msg);
  }
}

void StandardRobotPpRos2Node::publishImuData(ReceiveImuData & imu_data)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = now();
  // 下位机 IMU 安装在小 yaw / pitch 这套可动云台上，
  // 会同时跟随 `gimbal_yaw` 和 `gimbal_pitch` 运动，因此 frame_id 对齐到 `gimbal_pitch`。
  imu_msg.header.frame_id = "gimbal_pitch";

  // Convert Euler angles to quaternion
  tf2::Quaternion q;
  q.setRPY(imu_data.data.roll, imu_data.data.pitch, imu_data.data.yaw);
  imu_msg.orientation = tf2::toMsg(q);
  imu_msg.angular_velocity.x = imu_data.data.roll_vel;
  imu_msg.angular_velocity.y = imu_data.data.pitch_vel;
  imu_msg.angular_velocity.z = imu_data.data.yaw_vel;
  imu_pub_->publish(imu_msg);

  if (publish_imu_as_gimbal_joint_state_) {
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = imu_msg.header.stamp;
    joint_msg.name = {"gimbal_yaw_joint", "gimbal_pitch_joint"};
    joint_msg.position = {imu_data.data.yaw, imu_data.data.pitch};
    joint_state_pub_->publish(joint_msg);
  }
}

void StandardRobotPpRos2Node::publishRobotInfo(ReceiveRobotInfoData & robot_info)
{
  ats_rm_interfaces::msg::RobotStateInfo msg;

  msg.header.stamp.sec = robot_info.time_stamp / 1000;
  msg.header.stamp.nanosec = (robot_info.time_stamp % 1000) * 1e6;
  msg.header.frame_id = "odom";

  msg.models.chassis = robot_models_.chassis.at(robot_info.data.type.chassis);
  msg.models.gimbal = robot_models_.gimbal.at(robot_info.data.type.gimbal);
  msg.models.shoot = robot_models_.shoot.at(robot_info.data.type.shoot);
  msg.models.arm = robot_models_.arm.at(robot_info.data.type.arm);
  msg.models.custom_controller =
    robot_models_.custom_controller.at(robot_info.data.type.custom_controller);

  robot_state_info_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishEventData(ReceiveEventData & event_data)
{
  ats_rm_interfaces::msg::EventData msg;

  msg.non_overlapping_supply_zone = event_data.data.non_overlapping_supply_zone;
  msg.overlapping_supply_zone = event_data.data.overlapping_supply_zone;
  msg.supply_zone = event_data.data.supply_zone;

  msg.small_energy = event_data.data.small_energy;
  msg.big_energy = event_data.data.big_energy;

  msg.central_highland = event_data.data.central_highland;
  msg.trapezoidal_highland = event_data.data.trapezoidal_highland;

  msg.center_gain_zone = event_data.data.center_gain_zone;

  event_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishAllRobotHp(ReceiveAllRobotHpData & all_robot_hp)
{
  ats_rm_interfaces::msg::GameRobotHP msg;

  msg.red_1_robot_hp = all_robot_hp.data.red_1_robot_hp;
  msg.red_2_robot_hp = all_robot_hp.data.red_2_robot_hp;
  msg.red_3_robot_hp = all_robot_hp.data.red_3_robot_hp;
  msg.red_4_robot_hp = all_robot_hp.data.red_4_robot_hp;
  msg.red_7_robot_hp = all_robot_hp.data.red_7_robot_hp;
  msg.red_outpost_hp = all_robot_hp.data.red_outpost_hp;
  msg.red_base_hp = all_robot_hp.data.red_base_hp;

  msg.blue_1_robot_hp = all_robot_hp.data.blue_1_robot_hp;
  msg.blue_2_robot_hp = all_robot_hp.data.blue_2_robot_hp;
  msg.blue_3_robot_hp = all_robot_hp.data.blue_3_robot_hp;
  msg.blue_4_robot_hp = all_robot_hp.data.blue_4_robot_hp;
  msg.blue_7_robot_hp = all_robot_hp.data.blue_7_robot_hp;
  msg.blue_outpost_hp = all_robot_hp.data.blue_outpost_hp;
  msg.blue_base_hp = all_robot_hp.data.blue_base_hp;

  all_robot_hp_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGameStatus(ReceiveGameStatusData & game_status)
{
  ats_rm_interfaces::msg::GameStatus msg;
  msg.game_progress = game_status.data.game_progress;
  msg.stage_remain_time = game_status.data.stage_remain_time;
  game_status_pub_->publish(msg);
  const uint8_t previous_progress = previous_game_progress_;
  const bool progress_changed = msg.game_progress != previous_progress;

  if (
    progress_changed ||
    msg.stage_remain_time != last_logged_stage_remain_time_)
  {
    const bool should_log_remain = last_logged_stage_remain_time_ < 0 ||
      (msg.stage_remain_time / 30) != (last_logged_stage_remain_time_ / 30) ||
      std::abs(msg.stage_remain_time - last_logged_stage_remain_time_) >= 10;
    if (progress_changed || should_log_remain) {
      RCLCPP_INFO(
        get_logger(),
        "[serial/referee] game_status progress=%s(%u) remain=%ds",
        gameProgressName(msg.game_progress), msg.game_progress, msg.stage_remain_time);
    }
  } else {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[serial/referee] game_status progress=%s(%u) remain=%ds",
      gameProgressName(msg.game_progress), msg.game_progress, msg.stage_remain_time);
  }
  previous_game_progress_ = msg.game_progress;
  last_logged_stage_remain_time_ = msg.stage_remain_time;

  if (record_rosbag_ && progress_changed) {
    RCLCPP_INFO(get_logger(), "Game progress: %d", game_status.data.game_progress);

    std::string service_name;
    switch (game_status.data.game_progress) {
      case ats_rm_interfaces::msg::GameStatus::COUNT_DOWN:
        service_name = "start_recording";
        break;
      case ats_rm_interfaces::msg::GameStatus::GAME_OVER:
        service_name = "stop_recording";
        break;
      default:
        return;
    }

    if (!callTriggerService(service_name)) {
      RCLCPP_ERROR(get_logger(), "Failed to call service: %s", service_name.c_str());
    }
  }
}

void StandardRobotPpRos2Node::publishRobotMotion(ReceiveRobotMotionData & robot_motion)
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = robot_motion.data.speed_vector.vx;
  msg.linear.y = robot_motion.data.speed_vector.vy;
  msg.angular.z = robot_motion.data.speed_vector.wz;

  robot_motion_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGroundRobotPosition(
  ReceiveGroundRobotPosition & ground_robot_position)
{
  ats_rm_interfaces::msg::GroundRobotPosition msg;

  msg.hero_position.x = ground_robot_position.data.hero_x;
  msg.hero_position.y = ground_robot_position.data.hero_y;

  msg.engineer_position.x = ground_robot_position.data.engineer_x;
  msg.engineer_position.y = ground_robot_position.data.engineer_y;

  msg.standard_3_position.x = ground_robot_position.data.standard_3_x;
  msg.standard_3_position.y = ground_robot_position.data.standard_3_y;

  msg.standard_4_position.x = ground_robot_position.data.standard_4_x;
  msg.standard_4_position.y = ground_robot_position.data.standard_4_y;

  ground_robot_position_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRfidStatus(ReceiveRfidStatus & rfid_status)
{
  ats_rm_interfaces::msg::RfidStatus msg;

  msg.base_gain_point = rfid_status.data.base_gain_point;
  msg.central_highland_gain_point = rfid_status.data.central_highland_gain_point;
  msg.enemy_central_highland_gain_point = rfid_status.data.enemy_central_highland_gain_point;
  msg.friendly_trapezoidal_highland_gain_point =
    rfid_status.data.friendly_trapezoidal_highland_gain_point;
  msg.enemy_trapezoidal_highland_gain_point =
    rfid_status.data.enemy_trapezoidal_highland_gain_point;
  msg.friendly_fly_ramp_front_gain_point = rfid_status.data.friendly_fly_ramp_front_gain_point;
  msg.friendly_fly_ramp_back_gain_point = rfid_status.data.friendly_fly_ramp_back_gain_point;
  msg.enemy_fly_ramp_front_gain_point = rfid_status.data.enemy_fly_ramp_front_gain_point;
  msg.enemy_fly_ramp_back_gain_point = rfid_status.data.enemy_fly_ramp_back_gain_point;
  msg.friendly_central_highland_lower_gain_point =
    rfid_status.data.friendly_central_highland_lower_gain_point;
  msg.friendly_central_highland_upper_gain_point =
    rfid_status.data.friendly_central_highland_upper_gain_point;
  msg.enemy_central_highland_lower_gain_point =
    rfid_status.data.enemy_central_highland_lower_gain_point;
  msg.enemy_central_highland_upper_gain_point =
    rfid_status.data.enemy_central_highland_upper_gain_point;
  msg.friendly_highway_lower_gain_point = rfid_status.data.friendly_highway_lower_gain_point;
  msg.friendly_highway_upper_gain_point = rfid_status.data.friendly_highway_upper_gain_point;
  msg.enemy_highway_lower_gain_point = rfid_status.data.enemy_highway_lower_gain_point;
  msg.enemy_highway_upper_gain_point = rfid_status.data.enemy_highway_upper_gain_point;
  msg.friendly_fortress_gain_point = rfid_status.data.friendly_fortress_gain_point;
  msg.friendly_outpost_gain_point = rfid_status.data.friendly_outpost_gain_point;
  msg.friendly_supply_zone_non_exchange = rfid_status.data.friendly_supply_zone_non_exchange;
  msg.friendly_supply_zone_exchange = rfid_status.data.friendly_supply_zone_exchange;
  msg.friendly_big_resource_island = rfid_status.data.friendly_big_resource_island;
  msg.enemy_big_resource_island = rfid_status.data.enemy_big_resource_island;
  msg.center_gain_point = rfid_status.data.center_gain_point;

  rfid_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRobotStatus(ReceiveRobotStatus & robot_status)
{
  ats_rm_interfaces::msg::RobotStatus msg;

  msg.robot_id = robot_status.data.robot_id;
  msg.robot_level = robot_status.data.robot_level;
  msg.current_hp = robot_status.data.current_hp;
  msg.maximum_hp = robot_status.data.maximum_hp;
  msg.shooter_barrel_cooling_value = robot_status.data.shooter_barrel_cooling_value;
  msg.shooter_barrel_heat_limit = robot_status.data.shooter_barrel_heat_limit;
  msg.shooter_17mm_1_barrel_heat = robot_status.data.shooter_17mm_1_barrel_heat;
  msg.robot_pos.position.x = robot_status.data.robot_pos_x;
  msg.robot_pos.position.y = robot_status.data.robot_pos_y;
  msg.robot_pos.orientation =
    tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_status.data.robot_pos_angle));
  msg.armor_id = robot_status.data.armor_id;
  msg.hp_deduction_reason = robot_status.data.hp_deduction_reason;
  msg.projectile_allowance_17mm = robot_status.data.projectile_allowance_17mm;
  msg.remaining_gold_coin = robot_status.data.remaining_gold_coin;

  msg.is_hp_deduced = last_hp_ >= 0.0F && (last_hp_ - static_cast<float>(msg.current_hp) > 0.0F);
  last_hp_ = robot_status.data.current_hp;

  robot_status_pub_->publish(msg);

  const bool status_changed =
    last_logged_current_hp_ != static_cast<int>(msg.current_hp) ||
    last_logged_maximum_hp_ != static_cast<int>(msg.maximum_hp) ||
    last_logged_projectile_allowance_17mm_ != static_cast<int>(msg.projectile_allowance_17mm) ||
    last_logged_heat_ != static_cast<int>(msg.shooter_17mm_1_barrel_heat) ||
    last_logged_armor_id_ != static_cast<int>(msg.armor_id) ||
    last_logged_hp_reason_ != static_cast<int>(msg.hp_deduction_reason) ||
    last_logged_is_hp_deduced_ != msg.is_hp_deduced;

  if (status_changed) {
    RCLCPP_INFO(
      get_logger(),
      "[serial/referee] robot_status id=%u hp=%u/%u ammo=%u heat=%u deduced=%d armor_id=%u reason=%s(%u)",
      msg.robot_id, msg.current_hp, msg.maximum_hp, msg.projectile_allowance_17mm,
      msg.shooter_17mm_1_barrel_heat, static_cast<int>(msg.is_hp_deduced), msg.armor_id,
      hpDeductionReasonName(msg.hp_deduction_reason), msg.hp_deduction_reason);
    last_logged_current_hp_ = static_cast<int>(msg.current_hp);
    last_logged_maximum_hp_ = static_cast<int>(msg.maximum_hp);
    last_logged_projectile_allowance_17mm_ = static_cast<int>(msg.projectile_allowance_17mm);
    last_logged_heat_ = static_cast<int>(msg.shooter_17mm_1_barrel_heat);
    last_logged_armor_id_ = static_cast<int>(msg.armor_id);
    last_logged_hp_reason_ = static_cast<int>(msg.hp_deduction_reason);
    last_logged_is_hp_deduced_ = msg.is_hp_deduced;
  } else {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[serial/referee] robot_status id=%u hp=%u/%u ammo=%u heat=%u deduced=%d",
      msg.robot_id, msg.current_hp, msg.maximum_hp, msg.projectile_allowance_17mm,
      msg.shooter_17mm_1_barrel_heat, static_cast<int>(msg.is_hp_deduced));
  }

  if (set_detector_color_) {
    uint8_t detect_color;
    if (getDetectColor(robot_status.data.robot_id, detect_color)) {
      if (!initial_set_param_ || detect_color != previous_receive_color_) {
        previous_receive_color_ = detect_color;
        setParam(rclcpp::Parameter("detect_color", detect_color));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }
}

void StandardRobotPpRos2Node::publishJointState(ReceiveJointState & packet)
{
  double small_yaw = packet.data.small_yaw;
  if (!small_yaw_is_relative_) {
    small_yaw -= packet.data.big_yaw;
  }
  if (invert_small_yaw_) {
    small_yaw = -small_yaw;
  }
  small_yaw = normalizeAngle(small_yaw + small_yaw_offset_);

  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = now();
  joint_msg.name = {
    "gimbal_yaw_odom_joint",
    "gimbal_yaw_joint",
    "gimbal_pitch_joint",
  };
  joint_msg.position = {
    packet.data.big_yaw,
    small_yaw,
    packet.data.pitch,
  };
  joint_state_pub_->publish(joint_msg);
}

void StandardRobotPpRos2Node::publishLegacyJointState(ReceiveLegacyJointState & packet)
{
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = now();
  joint_msg.name = {"gimbal_yaw_odom_joint"};
  joint_msg.position = {packet.data.yaw};
  joint_state_pub_->publish(joint_msg);

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Received legacy two-axis joint state. Only big yaw is used for TF; upgrade lower controller to publish big_yaw, small_yaw, pitch for accurate vision target projection.");
}

void StandardRobotPpRos2Node::publishBuff(ReceiveBuff & buff)
{
  ats_rm_interfaces::msg::Buff msg;
  msg.recovery_buff = buff.data.recovery_buff;
  msg.cooling_buff = buff.data.cooling_buff;
  msg.defence_buff = buff.data.defence_buff;
  msg.vulnerability_buff = buff.data.vulnerability_buff;
  msg.attack_buff = buff.data.attack_buff;
  msg.remaining_energy = buff.data.remaining_energy;
  buff_pub_->publish(msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start sendData!");

  {
    std::lock_guard<std::mutex> lock(send_cmd_mutex_);
    send_robot_cmd_data_.frame_header.sof = SOF_SEND;
    send_robot_cmd_data_.frame_header.id = ID_ROBOT_CMD;
    send_robot_cmd_data_.frame_header.len = sizeof(SendRobotCmdData) - 6;
    send_robot_cmd_data_.data.speed_vector.vx = 0;
    send_robot_cmd_data_.data.speed_vector.vy = 0;
    send_robot_cmd_data_.data.speed_vector.wz = 0;
    send_robot_cmd_data_.data.speed_vector.mode =
      static_cast<decltype(send_robot_cmd_data_.data.speed_vector.mode)>(0);
    // 添加帧头crc8校验
    crc8::append_CRC8_check_sum(
      reinterpret_cast<uint8_t *>(&send_robot_cmd_data_), sizeof(HeaderFrame));
  }

  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      SendRobotCmdData send_packet;
      {
        std::lock_guard<std::mutex> lock(send_cmd_mutex_);
        // 每拍复查授权与看门狗。归零时必须同时置 speed_vector.stop：
        // 只清零 vx/vy/wz 而 stop 仍为 false，下位机看到的是「合法的零速度指令」，
        // 而不是「上层要求停机」，两者在固件侧的处理不同。
        CmdVelGateOutput output;
        if (cmd_vel_gate_.tick(std::chrono::steady_clock::now(), output)) {
          applyGateOutputLocked(output);
          if (output.reason != last_gate_reason_) {
            RCLCPP_WARN(
              get_logger(),
              "出口归零，原因码 %u（0=转发 1=瞬时零保持 2=授权归零 3=未授权 "
              "4=看门狗超时 5=链路断开）",
              static_cast<unsigned>(output.reason));
            last_gate_reason_ = output.reason;
          }
        }
        send_packet = send_robot_cmd_data_;
      }
      send_packet.time_stamp = static_cast<uint32_t>(this->now().nanoseconds() / 1000000ULL);

      // 整包数据校验
      // 添加数据段crc16校验
      crc16::append_CRC16_check_sum(
        reinterpret_cast<uint8_t *>(&send_packet), sizeof(SendRobotCmdData));

      // 发送数据
      std::vector<uint8_t> send_data = toVector(send_packet);
      serial_driver_->port()->send(send_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      setSerialLinkState(false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(send_cmd_mutex_);
  // 车体系全向语义不变：linear.x -> vx，linear.y -> vy，angular.z -> wz，
  // 出口不做任何坐标旋转、不做任何增益。
  const CmdVelGateOutput output = cmd_vel_gate_.onCmdVel(
    msg->linear.x, msg->linear.y, msg->angular.z, std::chrono::steady_clock::now());
  applyGateOutputLocked(output);
  if (output.reason == CmdVelGateReason::HOLD_TRANSIENT_ZERO) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "判定为通信抖动的瞬时零输入，保持最近一次非零速度（窗口上界 %d ms）。"
      "授权归零、急停、断连与看门狗均不走这条分支。",
      transient_zero_cmd_hold_timeout_ms_);
  }
  if (output.reason != last_gate_reason_) {
    if (output.reason != CmdVelGateReason::FORWARD) {
      RCLCPP_WARN(get_logger(), "出口状态切换到原因码 %u", static_cast<unsigned>(output.reason));
    }
    last_gate_reason_ = output.reason;
  }
}

void StandardRobotPpRos2Node::executionCommandCallback(
  const ats_navigation_interfaces::msg::ExecutionCommand::SharedPtr msg)
{
  using ExecutionCommand = ats_navigation_interfaces::msg::ExecutionCommand;
  const bool execute = msg->mode == ExecutionCommand::MODE_EXECUTE;
  // 用挂钟比较头时间戳年龄：transient_local 会把重启前的旧样本重投给新订阅者，
  // 直接接受会让底盘在没有当前授权的情况下恢复运动。
  const rclcpp::Time stamp(msg->header.stamp, now().get_clock_type());
  const auto age = std::chrono::milliseconds(
    static_cast<int64_t>(std::max(0.0, (now() - stamp).seconds() * 1000.0)));

  bool accepted = false;
  {
    std::lock_guard<std::mutex> lock(send_cmd_mutex_);
    accepted = cmd_vel_gate_.onExecutionCommand(
      execute, msg->command_sequence, msg->localization_epoch, msg->map_generation, age);
    if (!execute || !accepted) {
      // STOP 与被拒样本都立即归零，不等下一拍 cmd_vel。
      CmdVelGateOutput zero;
      zero.stop = true;
      zero.reason = execute ? CmdVelGateReason::NO_AUTHORIZATION
                            : CmdVelGateReason::AUTHORIZED_ZERO;
      applyGateOutputLocked(zero);
      last_gate_reason_ = zero.reason;
    }
  }

  if (!accepted) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "拒绝执行授权样本：sequence=%" PRIu64 "，年龄 %" PRId64 " ms（租约 %.0f ms）。"
      "序号必须严格递增且时间戳新鲜。",
      msg->command_sequence, static_cast<int64_t>(age.count()),
      execution_command_timeout_ * 1000.0);
    return;
  }
  if (!execute) {
    RCLCPP_WARN(
      get_logger(),
      "收到 ExecutionCommand STOP（sequence=%" PRIu64 "，failure_reason=%u）：出口立即归零。",
      msg->command_sequence, static_cast<unsigned>(msg->failure_reason));
  } else {
    RCLCPP_INFO(
      get_logger(),
      "执行授权已更新：sequence=%" PRIu64 "，localization_epoch=%" PRIu64
      "，map_generation=%" PRIu64,
      msg->command_sequence, msg->localization_epoch, msg->map_generation);
  }
}

void StandardRobotPpRos2Node::emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(send_cmd_mutex_);
    cmd_vel_gate_.onEmergencyStop(msg->data);
    if (msg->data) {
      CmdVelGateOutput zero;
      zero.stop = true;
      zero.reason = CmdVelGateReason::AUTHORIZED_ZERO;
      applyGateOutputLocked(zero);
      last_gate_reason_ = zero.reason;
    }
  }
  if (msg->data) {
    RCLCPP_WARN(get_logger(), "收到 emergency_stop=true：出口立即归零并撤销授权。");
  } else {
    // 单独的 false 不构成授权，这里只记录，不放行。
    RCLCPP_INFO(
      get_logger(),
      "收到 emergency_stop=false：仅清除急停标志，不恢复执行授权；"
      "恢复运动仍需新的 ExecutionCommand(EXECUTE)。");
  }
}

void StandardRobotPpRos2Node::applyGateOutputLocked(const CmdVelGateOutput & output)
{
  send_robot_cmd_data_.data.speed_vector.vx = static_cast<float>(output.vx);
  send_robot_cmd_data_.data.speed_vector.vy = static_cast<float>(output.vy);
  send_robot_cmd_data_.data.speed_vector.wz = static_cast<float>(output.wz);
  send_robot_cmd_data_.data.speed_vector.stop = output.stop;
}

rcl_interfaces::msg::SetParametersResult StandardRobotPpRos2Node::onParametersSet(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(send_cmd_mutex_);
  (void)params;

  return result;
}

void StandardRobotPpRos2Node::cmdGimbalJointCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() != msg->position.size()) {
    RCLCPP_ERROR(
      get_logger(), "JointState message name and position arrays are of different sizes");
    return;
  }

  std::lock_guard<std::mutex> lock(send_cmd_mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "gimbal_pitch_joint") {
      send_robot_cmd_data_.data.gimbal.pitch = msg->position[i];
    } else if (msg->name[i] == "gimbal_yaw_joint") {
      send_robot_cmd_data_.data.gimbal.yaw = msg->position[i];
    }
  }
}

void StandardRobotPpRos2Node::cmdShootCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(send_cmd_mutex_);
  send_robot_cmd_data_.data.shoot.fric_on = true;
  send_robot_cmd_data_.data.shoot.fire = msg->data;
}

void StandardRobotPpRos2Node::cmdRobotModeCallback(
  const example_interfaces::msg::UInt8::SharedPtr msg)
{
  constexpr uint8_t kMoveMode = 3;
  constexpr uint8_t kAttackMode = 1;
  constexpr uint8_t kDefendMode = 2;
  // 协议当前约定：
  // 3=move，1=attack，2=defend。
  // 这里不重新做业务判断，只做最终的合法值保护和串口结构体写入。

  // 当前协议只支持 1/2/3 三种姿态，收到非法值时自动回退到 move。
  const uint8_t mode =
    (msg->data == kAttackMode || msg->data == kDefendMode || msg->data == kMoveMode) ?
    msg->data :
    kMoveMode;
  if (mode != msg->data) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received unsupported robot mode %u, fallback to move mode", msg->data);
  }

  if (mode != previous_robot_mode_cmd_) {
    RCLCPP_INFO(
      get_logger(),
      "[serial/cmd] robot_mode request=%s(%u) raw=%u",
      robotModeName(mode), mode, msg->data);
    previous_robot_mode_cmd_ = mode;
  } else {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[serial/cmd] robot_mode=%s(%u)",
      robotModeName(mode), mode);
  }

  std::lock_guard<std::mutex> lock(send_cmd_mutex_);
  // 将行为树姿态模式写入串口发送结构体，后续由发送线程发给下位机。
  send_robot_cmd_data_.data.speed_vector.mode =
    static_cast<decltype(send_robot_cmd_data_.data.speed_vector.mode)>(mode);
}

void StandardRobotPpRos2Node::setParam(const rclcpp::Parameter & param)
{
  if (!initial_set_param_) {
    auto node_graph = this->get_node_graph_interface();
    auto node_names = node_graph->get_node_names();
    std::vector<std::string> possible_detectors = {
      "armor_detector_openvino", "armor_detector_opencv"};

    for (const auto & name : possible_detectors) {
      for (const auto & node_name : node_names) {
        if (node_name.find(name) != std::string::npos) {
          detector_node_name_ = node_name;
          break;
        }
      }
      if (!detector_node_name_.empty()) {
        break;
      }
    }

    if (detector_node_name_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No detector node found!");
      return;
    }

    detector_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, detector_node_name_);
    if (!detector_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *this->get_clock(), 1000, "Service not ready, skipping parameter set");
      return;
    }
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

bool StandardRobotPpRos2Node::getDetectColor(uint8_t robot_id, uint8_t & color)
{
  if (robot_id == 0 || (robot_id > 11 && robot_id < 101)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), 1000, "Invalid robot ID: %d. Color not set.", robot_id);
    return false;
  }
  color = (robot_id >= 100) ? 0 : 1;
  return true;
}

bool StandardRobotPpRos2Node::callTriggerService(const std::string & service_name)
{
  auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto start_time = std::chrono::steady_clock::now();
  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the service: %s", service_name.c_str());
      return false;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (elapsed_time > std::chrono::seconds(5)) {
      RCLCPP_ERROR(
        get_logger(), "Service %s not available after 5 seconds, giving up.", service_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }

  auto result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      get_logger(), "Service %s call succeeded: %s", service_name.c_str(),
      result.get()->success ? "true" : "false");
    return result.get()->success;
  }

  RCLCPP_ERROR(get_logger(), "Service %s call failed", service_name.c_str());
  return false;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)
