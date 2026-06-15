cmake_minimum_required(VERSION 3.8)
project(drone_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Core ROS 2 build tools ────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ── C++ dependencies ──────────────────────────────────────────────────────────
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# ── C++ safety filter node ────────────────────────────────────────────────────
add_executable(safety_filter_node src/safety_filter.cpp)
ament_target_dependencies(safety_filter_node
  rclcpp
  std_msgs
  px4_msgs
  nlohmann_json
)

# ── Install C++ executable ────────────────────────────────────────────────────
install(TARGETS safety_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python package registration ───────────────────────────────────────────────
# Requires: ros2_ws/src/drone_safety/drone_safety/__init__.py
ament_python_install_package(${PROJECT_NAME})

# ── Install Python nodes (each gets its own block due to RENAME) ──────────────
# ros2 run drone_safety benchmark_pipeline
install(PROGRAMS
  ../../../scripts/benchmark_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME benchmark_pipeline
)

# ros2 run drone_safety llm_gateway_node
install(PROGRAMS
  scripts/llm_gateway_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME llm_gateway_node
)

# ── Install data scripts ──────────────────────────────────────────────────────
# ros2 run drone_safety plot_results
install(PROGRAMS
  ../../../data/plot_results.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_results
)

# ── Install launch files ──────────────────────────────────────────────────────
# ros2 launch drone_safety safety_filter.launch.py
# ros2 launch drone_safety benchmark.launch.py
# ros2 launch drone_safety full_system.launch.py
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Install config files ──────────────────────────────────────────────────────
# Loaded at runtime by safety_filter_node and llm_gateway_node
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
