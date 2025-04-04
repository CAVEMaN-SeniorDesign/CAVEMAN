cmake_minimum_required(VERSION 3.5)
project(rover)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(four_wheel_steering_msgs REQUIRED)

add_executable(talker src/sample_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/sample_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(cmd_vel_to_joints src/cmd_vel_to_joints.cpp)
ament_target_dependencies(cmd_vel_to_joints rclcpp sensor_msgs geometry_msgs control_toolbox)

add_executable(cmd_vel_to_wheels src/cmd_vel_to_wheels.cpp)
ament_target_dependencies(cmd_vel_to_wheels rclcpp geometry_msgs std_msgs trajectory_msgs)

add_executable(joy_to_drive src/joy_to_drive.cpp)
ament_target_dependencies(joy_to_drive rclcpp sensor_msgs std_msgs trajectory_msgs)

install(
  TARGETS 
  cmd_vel_to_joints
  cmd_vel_to_wheels
  joy_to_drive
  talker 
  listener
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)






ament_package()
