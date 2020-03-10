cmake_minimum_required(VERSION 3.5)
project(yumi_dynamics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_wrapper REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)


add_library(external_force SHARED src/external_force.cpp)
target_include_directories(external_force PRIVATE include)
ament_target_dependencies(external_force
                          rclcpp
                          geometry_msgs
                          sensor_msgs
                          Eigen3
                          urdf
                          kdl_wrapper
)

add_executable(external_force_node src/external_force_node.cpp)
target_include_directories(external_force_node PRIVATE include)
target_link_libraries(external_force_node
                      external_force)
ament_target_dependencies(external_force_node
                          kdl_wrapper
                          orocos_kdl
                          rclcpp
                          urdf
                          kdl_wrapper
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS 
  external_force_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
        external_force
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries( external_force )
ament_package()