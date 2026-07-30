cmake_minimum_required(VERSION 3.8)
project(ylr1d_hmi)

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

# ── Lite version (no RViz2) ──────────────────────────────
add_executable(ylr1d_hmi
  src/main.cpp
  src/hmi_window.cpp
  include/ylr1d_hmi/hmi_window.hpp
)
target_include_directories(ylr1d_hmi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ylr1d_hmi PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi
  Qt5::Widgets
  Qt5::Core
)
ament_target_dependencies(ylr1d_hmi
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

# ── RViz2 version ────────────────────────────────────────
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)

add_executable(ylr1d_hmi_rviz
  src/main_rviz.cpp
  src/hmi_window_rviz.cpp
  src/hmi_window.cpp
  include/ylr1d_hmi/hmi_window.hpp
  include/ylr1d_hmi/hmi_window_rviz.hpp
)
target_include_directories(ylr1d_hmi_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ylr1d_hmi_rviz PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_rviz
  Qt5::Widgets
  Qt5::Core
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
)
ament_target_dependencies(ylr1d_hmi_rviz
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

# ── Install both ─────────────────────────────────────────
install(TARGETS ylr1d_hmi ylr1d_hmi_rviz DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
