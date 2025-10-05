cmake_minimum_required(VERSION 3.8)
project(robot_behaviour)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp sensor_msgs)

install(TARGETS obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME})

ament_package()
