cmake_minimum_required(VERSION 3.5)
project(lalosoft_robot_control_host)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lalosoft_robot_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime lalosoft_robot_msgs)


add_executable(control_host robot_control_host.cpp)
ament_target_dependencies(control_host rclcpp std_msgs sensor_msgs lalosoft_robot_msgs)


install(TARGETS
  control_host
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
