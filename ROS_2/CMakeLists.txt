cmake_minimum_required(VERSION 3.8)

project(ROS_2)
find_package(rclcpp REQUIRED)

add_executable(main main.cpp)

target_include_directories(main PUBLIC
  ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(main
  ${rclcpp_LIBRARIES}
)