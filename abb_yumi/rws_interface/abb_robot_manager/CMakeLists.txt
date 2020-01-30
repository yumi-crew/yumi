cmake_minimum_required(VERSION 3.5)
project(abb_robot_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

find_package(abb_librws REQUIRED)
find_package(abb_robot_manager_interfaces REQUIRED)
find_package(Poco 1.4.3 REQUIRED
             COMPONENTS Net
                        Util
                        Foundation
                        XML)


# yumi_robot_manager
add_library(yumi_robot_manager SHARED src/yumi_robot_manager.cpp)
target_include_directories(yumi_robot_manager PRIVATE include ${Poco_LIBRARIES} )
ament_target_dependencies(yumi_robot_manager
                          rcutils
                          rclcpp
                          abb_librws
                          abb_robot_manager_interfaces
                          poco
)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions( yumi_robot_manager PRIVATE
                           "ABB_ROBOT_MANAGER_BUILDING_DLL"
)

# yumi_robot_manager_sim
add_library(yumi_robot_manager_sim SHARED src/yumi_robot_manager_sim.cpp)
target_include_directories(yumi_robot_manager_sim PRIVATE include )
ament_target_dependencies(yumi_robot_manager_sim
                          rcutils
                          rclcpp
                          abb_robot_manager_interfaces
)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions( yumi_robot_manager_sim PRIVATE
                           "ABB_ROBOT_MANAGER_BUILDING_DLL"
)


# yumi_robot_manager_node
add_executable(yumi_robot_manager_node src/yumi_robot_manager_node.cpp)
target_include_directories(yumi_robot_manager_node PRIVATE include )
target_link_libraries(yumi_robot_manager_node yumi_robot_manager ${Poco_LIBRARIES} )
ament_target_dependencies(yumi_robot_manager_node
                          rclcpp
                          abb_librws
                          abb_robot_manager_interfaces
                          rcutils
                          poco
) 

# yumi_robot_manager_sim_node
add_executable(yumi_robot_manager_sim_node src/yumi_robot_manager_sim_node.cpp)
target_include_directories(yumi_robot_manager_sim_node PRIVATE include )
target_link_libraries(yumi_robot_manager_sim_node yumi_robot_manager_sim )
ament_target_dependencies(yumi_robot_manager_sim_node
                          rclcpp
                          abb_robot_manager_interfaces
                          rcutils
) 

                          
install(DIRECTORY include/ DESTINATION include)


install(TARGETS yumi_robot_manager yumi_robot_manager_sim
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS 
        yumi_robot_manager_node yumi_robot_manager_sim_node
        DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(abb_robot_manager_interfaces)


ament_package()                          