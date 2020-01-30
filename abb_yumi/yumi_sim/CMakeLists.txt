cmake_minimum_required(VERSION 3.5)
project(yumi_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)

add_executable(global_joint_state_node src/global_joint_state_node.cpp)
ament_target_dependencies(global_joint_state_node 
                          rclcpp                            
                          sensor_msgs
                          angles                               
)


install(TARGETS 
  global_joint_state_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()