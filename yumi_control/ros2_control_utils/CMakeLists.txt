cmake_minimum_required(VERSION 3.5)
project(ros2_control_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(sg_control_interfaces REQUIRED)

# pid
add_library(pid SHARED src/pid.cpp)
target_include_directories(pid PUBLIC include)
ament_target_dependencies(pid
                          rclcpp
)
# global_joint_state_node
add_executable(global_joint_state_node src/global_joint_state_node.cpp)
ament_target_dependencies(global_joint_state_node 
                          rclcpp                            
                          sensor_msgs
                          angles
                          sg_control_interfaces                               
)
# global_joint_state_node_sim
add_executable(global_joint_state_node_sim src/global_joint_state_node_sim.cpp)
ament_target_dependencies(global_joint_state_node_sim 
                          rclcpp                            
                          sensor_msgs
                          angles
                          sg_control_interfaces                               
)

install(TARGETS
  pid
  global_joint_state_node
  global_joint_state_node_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  pid
  global_joint_state_node
  global_joint_state_node_sim
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories( include )
ament_export_libraries( pid )
ament_package()
