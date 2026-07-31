cmake_minimum_required(VERSION 3.8)
project(ylr1d_position_simulate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── 公共库: PID + JointSimulator + JointGroup ─────
add_library(ylr1d_simulate_common
  src/core/pid.cpp
  src/core/joint_simulator.cpp
  src/groups/joint_group.cpp
)
target_include_directories(ylr1d_simulate_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ylr1d_simulate_common PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ylr1d_simulate_common
  "rclcpp" "std_msgs" "sensor_msgs")

# ── chassis_simulate ──────────────────────────────
add_executable(chassis_simulate
  src/nodes/chassis_simulate_node.cpp
  src/main_chassis.cpp
)
target_link_libraries(chassis_simulate ylr1d_simulate_common)
ament_target_dependencies(chassis_simulate
  "rclcpp" "std_msgs" "sensor_msgs")

# ── arm_simulate ──────────────────────────────────
add_executable(arm_simulate
  src/nodes/arm_simulate_node.cpp
  src/main_arm.cpp
)
target_link_libraries(arm_simulate ylr1d_simulate_common)
ament_target_dependencies(arm_simulate
  "rclcpp" "std_msgs" "sensor_msgs")

install(TARGETS
  chassis_simulate arm_simulate
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
