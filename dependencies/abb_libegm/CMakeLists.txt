cmake_minimum_required(VERSION 3.5)
project(abb_libegm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

# ##############################################################################
# Google Protocol Buffers ##
# ##############################################################################
find_package(Protobuf REQUIRED)

# work around Protobuf exporting 'lpthread' as a library: we export the
# dependency on pthread using the CFG_EXTRAS files
list(REMOVE_ITEM PROTOBUF_LIBRARIES "-lpthread")

# Make sure protoc is present, as apparently the above find_package() doesn't
# check that.
find_program(Protobuf_PROTOC_LOC NAMES protoc)
if(NOT Protobuf_PROTOC_LOC)
  message(
    FATAL_ERROR
      "Cannot find required 'protoc', cannot process Protobuf files without it. Aborting."
    )
endif()

# Generate C++ for protocol classes (headers and sources get written to the
# CMAKE_CURRENT_BINARY_DIR location).
set(EgmProtoFiles proto/egm.proto proto/egm_wrapper.proto
                  proto/egm_wrapper_trajectory.proto)
if(NOT QUIET)
  message(STATUS "Generating protobuf C++ for: ${EgmProtoFiles}")
endif()
protobuf_generate_cpp(EgmProtoSources EgmProtoHeaders ${EgmProtoFiles})

# ##############################################################################
# Build ##
# ##############################################################################
set(SRC_FILES
    src/egm_base_interface.cpp
    src/egm_common.cpp
    src/egm_common_auxiliary.cpp
    src/egm_controller_interface.cpp
    src/egm_interpolator.cpp
    src/egm_logger.cpp
    src/egm_udp_server.cpp
    src/egm_trajectory_interface.cpp
    ${EgmProtoSources})

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
include_directories(include
                    ${CMAKE_CURRENT_BINARY_DIR} # Contains protobuf generated
                                                # sources
                    ${Boost_INCLUDE_DIRS}
                    ${PROTOBUF_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${EgmProtoHeaders} DESTINATION include/${PROJECT_NAME})
install(FILES ${EgmProtoFiles} DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Boost Protobuf)

ament_package(CONFIG_EXTRAS cmake/abb_libegm-extras.cmake.in)
