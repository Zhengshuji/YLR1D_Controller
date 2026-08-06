cmake_minimum_required(VERSION 3.8)
project(ylr1d_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ylr1d_algorithm_sim REQUIRED)

# ── chassis_control ──────────────────────────────
add_executable(chassis_control
  src/nodes/chassis_control_node.cpp
  src/groups/group_forwarder.cpp
  src/main_chassis.cpp
)
target_include_directories(chassis_control PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(chassis_control ylr1d_algorithm_sim::ylr1d_algorithm_sim)
ament_target_dependencies(chassis_control
  "rclcpp" "std_msgs" "sensor_msgs")

# ── arm_control ──────────────────────────────────
add_executable(arm_control
  src/nodes/arm_control_node.cpp
  src/groups/group_forwarder.cpp
  src/main_arm.cpp
)
target_include_directories(arm_control PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(arm_control ylr1d_algorithm_sim::ylr1d_algorithm_sim)
ament_target_dependencies(arm_control
  "rclcpp" "std_msgs" "sensor_msgs")

install(TARGETS
  chassis_control arm_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
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
