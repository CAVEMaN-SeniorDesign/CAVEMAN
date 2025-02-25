cmake_minimum_required(VERSION 3.5)
project(rover_comms)

# Use C++20 for compatibility with Ben & Kush's code
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

set(CMAKE_PREFIX_PATH "/usr/local/lib")
set(HEADER_DIR ${CMAKE_PREFIX_PATH}/../include/c++)
set(COMMON_DIR ${CMAKE_PREFIX_PATH}/../include/common)
set(PROTOBUF_DIR ${CMAKE_PREFIX_PATH}/../include/c++/include)
set(JETSON_CAVE_TALK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Jetson-Comms)
set(CAVETALK_BUILD_DIR ${JETSON_CAVE_TALK_DIR}/CAVeTalk/build) 

add_subdirectory(${JETSON_CAVE_TALK_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED) 
find_package(rover_interfaces REQUIRED)

add_executable(rover_comm src/rover_comm.cpp)
ament_target_dependencies(rover_comm rclcpp sensor_msgs std_msgs rover_interfaces)

target_include_directories(rover_comm 
    PUBLIC
        ${COMMON_DIR}
        ${HEADER_DIR}
        ${JETSON_CAVE_TALK_DIR}/inc
        ${CAVETALK_BUILD_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(rover_comm 
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-common.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c_messages.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp_messages.a
        jetson_comms
)

# Install
install(
  TARGETS rover_comm
  DESTINATION lib/${PROJECT_NAME}
)

# Build tests if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
