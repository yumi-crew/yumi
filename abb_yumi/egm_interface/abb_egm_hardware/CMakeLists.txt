cmake_minimum_required(VERSION 3.5)
project(abb_egm_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(abb_libegm REQUIRED)

find_package(parameter_server_interfaces REQUIRED)



add_library(abb_egm_hardware SHARED src/abb_egm_hardware.cpp)
target_include_directories(abb_egm_hardware PRIVATE include)
ament_target_dependencies(abb_egm_hardware
                          angles
                          rcutils
                          abb_libegm
                          controller_manager
                          controller_interface
                          hardware_interface 
                          parameter_server_interfaces)
# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(abb_egm_hardware PRIVATE
                           "ABB_EGM_HARDWARE_BUILDING_DLL")

add_library(abb_egm_hardware_sim SHARED src/abb_egm_hardware_sim.cpp)
target_include_directories(abb_egm_hardware_sim PRIVATE include)
ament_target_dependencies(abb_egm_hardware_sim
                          angles
                          rcutils
                          abb_libegm
                          controller_manager
                          controller_interface
                          hardware_interface)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(abb_egm_hardware_sim PRIVATE
                           "ABB_EGM_HARDWARE_BUILDING_DLL")


add_executable(abb_egm_hardware_node src/abb_egm_hardware_node.cpp)
target_include_directories(abb_egm_hardware_node PRIVATE include)
target_link_libraries(abb_egm_hardware_node abb_egm_hardware ${Boost_LIBRARIES})
ament_target_dependencies(abb_egm_hardware_node
                          rclcpp
                          abb_libegm
                          hardware_interface)

add_executable(abb_egm_hardware_node_sim src/abb_egm_hardware_node_sim.cpp)
target_include_directories(abb_egm_hardware_node_sim PRIVATE include)
target_link_libraries(abb_egm_hardware_node_sim abb_egm_hardware_sim ${Boost_LIBRARIES})
ament_target_dependencies(abb_egm_hardware_node_sim
                          rclcpp
                          abb_libegm
                          hardware_interface)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS abb_egm_hardware abb_egm_hardware_sim
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS abb_egm_hardware_node
                abb_egm_hardware_node_sim
                DESTINATION
                lib/${PROJECT_NAME})

ament_package()
