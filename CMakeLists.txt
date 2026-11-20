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

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
