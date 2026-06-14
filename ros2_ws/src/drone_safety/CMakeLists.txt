cmake_minimum_required(VERSION 3.8)
project(drone_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Core ROS 2 build tool ─────────────────────────────────────────────────────
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

# ── Install Python nodes (makes them runnable via ros2 run) ───────────────────
# ros2 run drone_safety benchmark_pipeline
# ros2 run drone_safety llm_gateway_node  (once added)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/benchmark_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME benchmark_pipeline
)

# ── Install data scripts ──────────────────────────────────────────────────────
install(PROGRAMS
  data/plot_results.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_results
)

# ── Install launch files (once launch/ directory is added) ────────────────────
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Install config files (once config/ directory is added) ───────────────────
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
