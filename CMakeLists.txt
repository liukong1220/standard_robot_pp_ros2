cmake_minimum_required(VERSION 3.10)
project(standard_robot_pp_ros2)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
ament_auto_find_build_dependencies(REQUIRED
  nav2_msgs
  rclcpp_action
  tf2_geometry_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN standard_robot_pp_ros2::StandardRobotPpRos2Node
  EXECUTABLE standard_robot_pp_ros2_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN standard_robot_pp_ros2::GimbalManagerNode
  EXECUTABLE gimbal_manager_node
)

# 实车上 GimbalYawStatus 的唯一发布者。仿真的模拟 ack 只在 MuJoCo profile 存在。
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN standard_robot_pp_ros2::GimbalYawStatusBridgeNode
  EXECUTABLE gimbal_yaw_status_bridge_node
)


#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()

  # 出口授权与归零判据的功能测试。纯逻辑，不依赖串口设备，可在不通电时运行。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cmd_vel_authorization test/test_cmd_vel_authorization.cpp)
  target_include_directories(test_cmd_vel_authorization PRIVATE include)

  # 实车云台 ack 判据的功能测试。同样是纯逻辑，不需要云台上电。
  ament_add_gtest(test_gimbal_yaw_status_logic test/test_gimbal_yaw_status_logic.cpp)
  target_include_directories(test_gimbal_yaw_status_logic PRIVATE include)
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
