cmake_minimum_required(VERSION 3.5)
project(lalosoft_robot_ptz_host)
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
find_package(lalosoft_robot_msgs REQUIRED)
find_package(lalosoft_robot_shared REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

add_executable(ptz_host 
  robot_ptz_host.cpp
  robot_ptz_host.h 
  )
ament_target_dependencies(ptz_host rclcpp lalosoft_robot_shared lalosoft_robot_msgs WiringPi)
target_link_libraries(ptz_host ${WIRINGPI_LIBRARIES})


install(TARGETS
  ptz_host
  DESTINATION lib/${PROJECT_NAME})

ament_package()
