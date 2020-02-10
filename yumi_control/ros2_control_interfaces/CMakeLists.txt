cmake_minimum_required(VERSION 3.5)
project(ros2_control_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointControl.msg"
  "msg/PidParameters.msg"
  "srv/GetCurrentSimTime.srv"
  DEPENDENCIES std_msgs
)

# install(DESTINATION share/${PROJECT_NAME} )



ament_export_dependencies(rosidl_default_runtime)
ament_package()
