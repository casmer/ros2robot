cmake_minimum_required(VERSION 3.5)
project(lalosoft_robot_drive_host)

# Default to C++14
set(WiringPi_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/")
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
find_package(lalosoft_robot_msgs REQUIRED)
find_package(lalosoft_robot_shared REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(drive_host robot_drive_host.cpp)
ament_target_dependencies(drive_host rclcpp lalosoft_robot_msgs lalosoft_robot_shared WiringPi)
target_link_libraries(drive_host ${WIRINGPI_LIBRARIES})


install(TARGETS
  drive_host
  DESTINATION lib/${PROJECT_NAME})

ament_package()
