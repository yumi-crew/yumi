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


# robot_manager_client_node (TO BE REMOVED BY LIFECYCLE)
add_executable(robot_manager_client_node src/robot_manager_client_node.cpp)
target_include_directories(robot_manager_client_node PRIVATE include )
target_link_libraries(robot_manager_client_node robot_manager_client)
ament_target_dependencies(robot_manager_client_node
                          rcutils
                          rclcpp
                          yumi_robot_manager_interfaces                
) 

# grip_client
add_library(grip_client SHARED src/grip_client.cpp)
target_include_directories(grip_client PRIVATE include)
ament_target_dependencies(grip_client
                          rcutils
                          rclcpp
                          rclcpp_action
                          sg_control_interfaces
)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions( grip_client PRIVATE
                           "RWS_CLIENTS_BUILDING_DLL"
)                
# grip_client_node (TO BE REMOVED BY LIFECYCLE)
add_executable(grip_client_node src/grip_client_node.cpp)
target_include_directories(grip_client_node PRIVATE include )
target_link_libraries(grip_client_node grip_client)
ament_target_dependencies(grip_client_node
                          rcutils
                          rclcpp
                          rclcpp_action
                          sg_control_interfaces                        
)

                          
install(DIRECTORY include/ DESTINATION include)


install(TARGETS robot_manager_client grip_client
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS 
        robot_manager_client_node
        grip_client_node
        DESTINATION lib/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_dependencies(yumi_robot_manager_interfaces sg_control_interfaces)
ament_export_libraries(robot_manager_client grip_client)

ament_package()                          