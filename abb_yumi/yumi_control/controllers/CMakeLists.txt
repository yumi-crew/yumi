cmake_minimum_required(VERSION 3.5)
project(controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_parameter_server REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Required by joint_position_controller
find_package(ros2_control_utils REQUIRED)
find_package(ros2_control_interfaces REQUIRED)
find_package(parameter_server_interfaces REQUIRED)
find_package(angles REQUIRED)

add_library(
  default_controllers
  SHARED
  src/joint_trajectory_controller.cpp
  src/joint_position_controller.cpp
  src/joint_state_controller.cpp
  src/trajectory.cpp
)
target_include_directories(default_controllers PRIVATE include)
ament_target_dependencies(
  default_controllers
  "builtin_interfaces"
  "class_loader"
  "controller_interface"
  "controller_manager"
  "controller_parameter_server"
  "hardware_interface"
  "rclcpp"
  "rclcpp_lifecycle"
  "rcutils"
  "sensor_msgs"
  "trajectory_msgs"
  "ros2_control_utils"
  "ros2_control_interfaces"
  "parameter_server_interfaces"
  "angles"
)
controller_manager_register_controller(
  default_controllers
  "ros_controllers::JointStateController"
  "ros_controllers::JointTrajectoryController"
  "ros_controllers::JointPositionController"
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(default_controllers PRIVATE "ROS_CONTROLLERS_BUILDING_DLL")


install(DIRECTORY include/
        DESTINATION include
)

install(TARGETS default_controllers
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)


ament_export_dependencies(
  controller_interface
  rclcpp_lifecycle
  sensor_msgs
  trajectory_msgs
)

ament_export_include_directories( include )
ament_export_libraries( default_controllers )

ament_package()
