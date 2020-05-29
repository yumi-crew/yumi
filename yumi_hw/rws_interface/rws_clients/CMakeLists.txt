cmake_minimum_required(VERSION 3.5)
project(rws_clients)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yumi_robot_manager_interfaces REQUIRED)
find_package(sg_control_interfaces REQUIRED)


# robot_manager_client
add_library(robot_manager_client SHARED src/robot_manager_client.cpp)
target_include_directories(robot_manager_client PRIVATE include)
ament_target_dependencies(robot_manager_client
                          rcutils
                          rclcpp
                          yumi_robot_manager_interfaces               
)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions( robot_manager_client PRIVATE
                           "RWS_CLIENTS_BUILDING_DLL"
)

# gripper_client
add_library(gripper_client SHARED src/gripper_client.cpp)
target_include_directories(gripper_client PRIVATE include)
ament_target_dependencies(gripper_client
                          rcutils
                          rclcpp
                          rclcpp_action
                          sg_control_interfaces
)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions( gripper_client PRIVATE
                           "RWS_CLIENTS_BUILDING_DLL"
)                

                          
install(DIRECTORY include/ DESTINATION include)

install(TARGETS robot_manager_client gripper_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


ament_export_include_directories(include)
ament_export_dependencies(yumi_robot_manager_interfaces sg_control_interfaces)
ament_export_libraries(robot_manager_client gripper_client)

ament_package()                          