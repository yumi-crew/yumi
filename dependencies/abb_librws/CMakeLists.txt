cmake_minimum_required(VERSION 3.5)
project(abb_librws)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ##############################################################################
# POCO C++ Libraries ##
# ##############################################################################
# we need at least 1.4.3 because of websocket support
find_package(Poco 1.4.3 REQUIRED
             COMPONENTS Net
                        Util
                        Foundation
                        XML)

# ##############################################################################
# Build ##
# ##############################################################################
set(SRC_FILES
    src/rws_client.cpp
    src/rws_common.cpp
    src/rws_interface.cpp
    src/rws_poco_client.cpp
    src/rws_rapid.cpp
    src/rws_state_machine_interface.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE include ${Poco_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Poco_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
