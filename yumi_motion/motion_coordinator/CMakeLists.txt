cmake_minimum_required(VERSION 3.5)
project(motion_coordinator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit2_wrapper REQUIRED)
find_package(rws_clients REQUIRED)

# motion_coordinator
add_library(motion_coordinator src/motion_coordinator.cpp)
target_include_directories(motion_coordinator PRIVATE include)
ament_target_dependencies(motion_coordinator
                          moveit2_wrapper
                          rws_clients                                               
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS motion_coordinator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(motion_coordinator)
ament_export_include_directories(include)
ament_export_dependencies(moveit2_wrapper rws_clients)
ament_package()
