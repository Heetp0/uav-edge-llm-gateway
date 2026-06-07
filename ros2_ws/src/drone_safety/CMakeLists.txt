cmake_minimum_required(VERSION 3.8)
project(drone_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(safety_filter_node src/safety_filter.cpp)
ament_target_dependencies(safety_filter_node rclcpp std_msgs px4_msgs nlohmann_json)

install(TARGETS safety_filter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
