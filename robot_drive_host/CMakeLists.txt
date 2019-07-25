cmake_minimum_required(VERSION 3.5)
project(lalosoft_robot_drive_host)
set(robot_msgs_DIR "/home/casmer/harvey_robot_ws/install/share/robot_msgs/cmake")
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(WiringPi REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(drive_host robot_drive_host.cpp motor_hat/ugeek_motor_hat.cpp motor_hat/ugeek_motor_hat.hpp)
ament_target_dependencies(drive_host rclcpp std_msgs robot_msgs WiringPi)
target_link_libraries(drive_host ${WIRINGPI_LIBRARIES})


install(TARGETS
  drive_host
  DESTINATION lib/${PROJECT_NAME})

ament_package()
