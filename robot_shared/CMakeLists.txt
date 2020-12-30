cmake_minimum_required(VERSION 3.5)
project(lalosoft_robot_shared)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/FindWiringPi.cmake")

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
include_directories(
  include
)
ament_export_include_directories(robot_shared)
ament_export_dependencies(rosidl_default_runtime rclcpp)


add_library(robot_shared STATIC 
	motor_hat/ugeek_motor_hat.cpp  
	pwm/pca6895.cpp 
  	serial/mdds30.cpp 
  	)

ament_target_dependencies(robot_shared rclcpp)


install(TARGETS
  robot_shared
  DESTINATION lib/${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/)
  
install(FILES  
	pwm/pca6895.hpp
    DESTINATION include/${PROJECT_NAME}/
)

install(FILES 
	motor_hat/ugeek_motor_hat.hpp 
    DESTINATION include/${PROJECT_NAME}/
    
)

install(FILES 
	serial/mdds30.hpp
    DESTINATION include/${PROJECT_NAME}/
    
)

ament_package()