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
find_package(kdl_wrapper REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit2_wrapper REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(motion_coordinator REQUIRED)
find_package(pose_estimation REQUIRED)
find_package(pose_estimation_manager REQUIRED)
find_package(pose_estimation_interface REQUIRED)
find_package(zivid_interfaces REQUIRED)
find_package(Threads REQUIRED)

# Demo                           
add_executable(demo src/demo.cpp)
target_include_directories(demo PRIVATE include)
ament_target_dependencies(demo 
                          motion_coordinator
                          Boost
                          kdl_wrapper
                          rclcpp     
                          angles
                          pose_estimation_interface
                          zivid_interfaces
                          pose_estimation_manager
                          pose_estimation   
)

# Demo                           
add_executable(demo_dual src/demo_dual.cpp)
target_include_directories(demo_dual PRIVATE include)
ament_target_dependencies(demo_dual 
                          motion_coordinator
                          Boost
                          kdl_wrapper
                          rclcpp     
                          angles
                          pose_estimation_interface
                          zivid_interfaces
                          pose_estimation_manager
                          pose_estimation   
)


# test_motion_coordinator                         
add_executable(test_motion_coordinator src/test_motion_coordinator.cpp)
ament_target_dependencies(test_motion_coordinator 
                          motion_coordinator
                          Boost
                          kdl_wrapper                                    
)

# test_torque                         
add_executable(test_torque src/test_torque.cpp)
ament_target_dependencies(test_torque 
                          motion_coordinator
                          Boost
                          kdl_wrapper                                    
)

install(TARGETS 
  demo
  demo_dual
  test_motion_coordinator
  test_torque
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})


ament_package()