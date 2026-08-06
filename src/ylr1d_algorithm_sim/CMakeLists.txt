cmake_minimum_required(VERSION 3.8)
project(ylr1d_algorithm_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 定位工作空间 ThirdParty（Eigen vendored，仅构建期使用，不导出）
# 头文件在 <WS>/ThirdParty/Eigen/Eigen/Dense（从 /usr/include/eigen3 整体复制）
set(YLR1D_THIRD_PARTY "${CMAKE_CURRENT_SOURCE_DIR}/../../ThirdParty/Eigen")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── 算法层核心共享库（纯 C++ + Eigen，零 rclcpp 依赖） ──
# 目录约定（ROS2 规范）：include/ylr1d_algorithm_sim/ 头文件 + src/ 实现。
# 架构：虚基类(cooperative_controller / independent_controller / plant.hpp)
# + 按控制层分组的具名类(steering / wheel / torso / left_arm / right_arm)；
# 具体算法在 src/control_law/（pid / proportional / integrator），改算法只改 src。
add_library(ylr1d_algorithm_sim SHARED
  src/control_law/proportional.cpp
  src/control_law/pid.cpp
  src/control_law/integrator.cpp
  src/steering.cpp
  src/wheel.cpp
  src/torso.cpp
  src/left_arm.cpp
  src/right_arm.cpp
)
target_include_directories(ylr1d_algorithm_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${YLR1D_THIRD_PARTY}>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ylr1d_algorithm_sim PUBLIC cxx_std_17)

install(TARGETS ylr1d_algorithm_sim
  EXPORT ylr1d_algorithm_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(ylr1d_algorithm_sim)
ament_export_targets(ylr1d_algorithm_sim HAS_LIBRARY_TARGET)

# ── 仿真控制器组件库（ROS2 壳：订阅/发布 + 固定线程框架调用算法核心） ──
add_library(ylr1d_algorithm_components SHARED
  src/ros/sim_controller.cpp
)
target_include_directories(ylr1d_algorithm_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(ylr1d_algorithm_components ylr1d_algorithm_sim)
ament_target_dependencies(ylr1d_algorithm_components
  "rclcpp" "rclcpp_components" "std_msgs" "sensor_msgs")
target_compile_features(ylr1d_algorithm_components PUBLIC cxx_std_17)

rclcpp_components_register_node(ylr1d_algorithm_components
  PLUGIN "ylr1d_algorithm_sim::SteeringSimNode"
  EXECUTABLE sim_controller_steering)
rclcpp_components_register_node(ylr1d_algorithm_components
  PLUGIN "ylr1d_algorithm_sim::WheelSimNode"
  EXECUTABLE sim_controller_wheels)
rclcpp_components_register_node(ylr1d_algorithm_components
  PLUGIN "ylr1d_algorithm_sim::TorsoSimNode"
  EXECUTABLE sim_controller_torso)
rclcpp_components_register_node(ylr1d_algorithm_components
  PLUGIN "ylr1d_algorithm_sim::LeftArmSimNode"
  EXECUTABLE sim_controller_left_arm)
rclcpp_components_register_node(ylr1d_algorithm_components
  PLUGIN "ylr1d_algorithm_sim::RightArmSimNode"
  EXECUTABLE sim_controller_right_arm)

install(TARGETS ylr1d_algorithm_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid ylr1d_algorithm_sim)

  ament_add_gtest(test_plant test/test_plant.cpp)
  target_link_libraries(test_plant ylr1d_algorithm_sim)

  ament_add_gtest(test_closed_loop test/test_closed_loop.cpp)
  target_link_libraries(test_closed_loop ylr1d_algorithm_sim)

  ament_add_gtest(test_group test/test_group.cpp)
  target_link_libraries(test_group ylr1d_algorithm_sim)
endif()

ament_package()
