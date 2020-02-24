cmake_minimum_required(VERSION 3.5)
project(yumi_test)


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
find_package(rws_clients REQUIRED)
find_package(ros2_control_interfaces REQUIRED)
find_package(sg_control_interfaces REQUIRED)
find_package(yumi_robot_manager_interfaces REQUIRED)
find_package(kdl_test REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)

# find_package(pose_estimation REQUIRED)
# find_package(object_pose_estimator REQUIRED)
# find_package(pose_estimation_interface REQUIRED)
# find_package(zivid_interfaces REQUIRED)


# Sine wave 
add_executable(sine_wave src/sine_wave.cpp)
target_include_directories(sine_wave PRIVATE include)
ament_target_dependencies(sine_wave 
                          rclcpp                            
                          sensor_msgs
                          angles                             
                          ros2_control_interfaces
                          sg_control_interfaces
                          yumi_robot_manager_interfaces
                          rws_clients  
)
# Sine wave inf 
add_executable(sine_wave_inf src/sine_wave_inf.cpp)
target_include_directories(sine_wave_inf PRIVATE include)
ament_target_dependencies(sine_wave_inf 
                          rclcpp                            
                          sensor_msgs
                          angles                             
                          ros2_control_interfaces
                          sg_control_interfaces
                          yumi_robot_manager_interfaces
                          rws_clients   
)
# Step
add_executable(step src/step.cpp)
target_include_directories(step PRIVATE include)
ament_target_dependencies(step 
                          rclcpp                            
                          sensor_msgs
                          angles                             
                          ros2_control_interfaces
                          sg_control_interfaces
                          yumi_robot_manager_interfaces
                          rws_clients  
)
# Demo                           
add_executable(demo src/demo.cpp)
target_include_directories(demo PRIVATE include)
ament_target_dependencies(demo 
                          rclcpp                            
                          sensor_msgs
                          angles                             
                          ros2_control_interfaces 
                          sg_control_interfaces
                          yumi_robot_manager_interfaces
                          rws_clients 
)
# demo_kdl_ik                         
add_executable(demo_kdl_ik src/demo_kdl_ik.cpp)
target_include_directories(demo_kdl_ik PRIVATE include)
ament_target_dependencies(demo_kdl_ik 
                          rclcpp                            
                          sensor_msgs
                          angles                             
                          ros2_control_interfaces 
                          sg_control_interfaces
                          yumi_robot_manager_interfaces
                          rws_clients
                          kdl_test 
                          urdf
                          orocos_kdl
)
# graph_bringup                            
add_executable(graph_bringup src/graph_bringup.cpp)
target_include_directories(graph_bringup PRIVATE include)
ament_target_dependencies(graph_bringup 
                          rclcpp                            
                          sensor_msgs
                          angles                             
                          ros2_control_interfaces 
                          sg_control_interfaces
                          yumi_robot_manager_interfaces
                          rws_clients 
)

# demo_go_to_chessboard                      
# add_executable(demo_go_to_chessboard src/demo_go_to_chessboard.cpp)
# target_include_directories(demo_go_to_chessboard PRIVATE include)          
# ament_target_dependencies(demo_go_to_chessboard 
#                           rclcpp                            
#                           sensor_msgs
#                           angles                             
#                           ros2_control_interfaces 
#                           sg_control_interfaces
#                           yumi_robot_manager_interfaces
#                           rws_clients
#                           kdl_test 
#                           urdf
#                           orocos_kdl
#                           pose_estimation_interface
#                           zivid_interfaces
#                           object_pose_estimator
#                           pose_estimation
# )


install(DIRECTORY include/ DESTINATION include)

install(TARGETS 
  sine_wave
  sine_wave_inf
  step
  demo
  demo_kdl_ik
  graph_bringup
  # demo_go_to_chessboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()