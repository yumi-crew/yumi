cmake_minimum_required(VERSION 3.10)
project(parameter_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (for filesystem implementation)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(parameter_server_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "yaml-cpp dir ${YAML_CPP_INCLUDE_DIR}")
add_library(yaml_parser SHARED src/yaml_parser.cpp)
target_include_directories(yaml_parser PUBLIC include)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(yaml_parser PRIVATE "stdc++fs" yaml-cpp)
else()
  target_link_libraries(yaml_parser PRIVATE yaml-cpp)
endif()

add_library(ros2_control_parameter_server SHARED src/parameter_server.cpp)
target_include_directories(ros2_control_parameter_server PUBLIC include)
target_link_libraries(ros2_control_parameter_server yaml_parser)
ament_target_dependencies(
  ros2_control_parameter_server
  rclcpp
  parameter_server_interfaces
)

install(
  TARGETS ros2_control_parameter_server yaml_parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(param_server_exec src/param_server_exec.cpp)
target_link_libraries(param_server_exec ros2_control_parameter_server)
ament_target_dependencies(param_server_exec rclcpp controller_parameter_server)
ament_export_include_directories(
  include
)
install(TARGETS
  param_server_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
DESTINATION include)


ament_package()
