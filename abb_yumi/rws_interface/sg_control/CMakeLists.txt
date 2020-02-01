cmake_minimum_required(VERSION 3.5)
project(sg_control)

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
find_package(rclcpp_components REQUIRED)
find_package(abb_librws REQUIRED)
find_package(Poco 1.4.3 REQUIRED
             COMPONENTS Net
                        Util
                        Foundation
                        XML)
find_package(sg_control_interfaces REQUIRED)




# sg_control
add_library(sg_control SHARED src/sg_control.cpp)
target_include_directories(sg_control PRIVATE include ${Poco_LIBRARIES})
ament_target_dependencies(sg_control
                          rcutils
                          rclcpp
                          rclcpp_action
                          abb_librws
                          Poco 
                          rclcpp_components
                          sg_control_interfaces
)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions( sg_control PRIVATE
                           "SG_CONTROL_BUILDING_DLL")

# sg_control_sim                          
add_library(sg_control_sim SHARED src/sg_control_sim.cpp)
target_include_directories(sg_control_sim PRIVATE include)
ament_target_dependencies(sg_control_sim
                          rcutils
                          rclcpp
                          rclcpp_action
                          rclcpp_components
                          sg_control_interfaces
)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions( sg_control_sim PRIVATE
                          "SG_CONTROL_BUILDING_DLL")

# sg_control_node (TO BE REMOVED BY LIFECYCLE)
add_executable(sg_control_node src/sg_control_node.cpp)
target_include_directories(sg_control_node PRIVATE include )
target_link_libraries(sg_control_node sg_control ${Poco_LIBRARIES} )
ament_target_dependencies(sg_control_node
                          rclcpp
                          rclcpp_action
                          sg_control_interfaces                        
)
# sg_control_sim_node (TO BE REMOVED BY LIFECYCLE)
add_executable(sg_control_sim_node src/sg_control_sim_node.cpp)
target_include_directories(sg_control_sim_node PRIVATE include )
target_link_libraries(sg_control_sim_node sg_control_sim )
ament_target_dependencies(sg_control_sim_node
                          rclcpp
                          rclcpp_action
                          sg_control_interfaces                        
)


install(DIRECTORY include/ DESTINATION include)

install(TARGETS sg_control sg_control_sim
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS 
          sg_control_node
          sg_control_sim_node
          DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(sg_control_interfaces )

ament_package()                          