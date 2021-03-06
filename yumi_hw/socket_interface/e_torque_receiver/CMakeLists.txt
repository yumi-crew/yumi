cmake_minimum_required(VERSION 3.5)
project(e_torque_receiver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# e_torque_reciever
add_library(e_torque_receiver SHARED src/e_torque_receiver.cpp)
target_include_directories(e_torque_receiver PRIVATE include)
ament_target_dependencies(e_torque_receiver
                          rclcpp
                          sensor_msgs                                       
)

# e_torque_receiver_node
add_executable(e_torque_receiver_node src/e_torque_receiver_node.cpp)
target_include_directories(e_torque_receiver_node PRIVATE include)
target_link_libraries(e_torque_receiver_node PRIVATE e_torque_receiver)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS e_torque_receiver 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS e_torque_receiver_node
                DESTINATION
                lib/${PROJECT_NAME})

ament_export_libraries(e_torque_receiver)
ament_export_include_directories(include)
ament_package()
