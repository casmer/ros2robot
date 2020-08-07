cmake_minimum_required(VERSION 3.5)
project(lalosoft_robot_jazzy_drive_host)
set(robot_msgs_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../install/share/robot_msgs/cmake")
set(WiringPi_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/")
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/FindWiringPi.cmake")
find_package(WiringPi REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(drive_host 
  robot_drive_host.cpp 
  pwm/pca6895.cpp pwm/pca6895.hpp
  serial/mdds30.cpp serial/mdds30.hpp)
ament_target_dependencies(drive_host rclcpp robot_msgs WiringPi)
target_link_libraries(drive_host ${WIRINGPI_LIBRARIES})


install(TARGETS
  drive_host
  DESTINATION lib/${PROJECT_NAME})

ament_package()
