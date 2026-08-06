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
# 目录约定：先按算法分类（controller/ plant/），每算法再分 include/ 与 src/。
add_library(ylr1d_algorithm_sim SHARED
  controller/pid/src/pid.cpp
  controller/proportional/src/proportional_controller.cpp
  plant/position/src/position_plant.cpp
  plant/velocity/src/velocity_plant.cpp
  plant/position_group/src/position_group_plant.cpp
  plant/velocity_group/src/velocity_group_plant.cpp
)
target_include_directories(ylr1d_algorithm_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller/pid/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/controller/proportional/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plant/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plant/position/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plant/position_group/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plant/velocity/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plant/velocity_group/include>
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
install(DIRECTORY controller/include/ DESTINATION include)
install(DIRECTORY controller/pid/include/ DESTINATION include)
install(DIRECTORY controller/proportional/include/ DESTINATION include)
install(DIRECTORY plant/include/ DESTINATION include)
install(DIRECTORY plant/position/include/ DESTINATION include)
install(DIRECTORY plant/position_group/include/ DESTINATION include)
install(DIRECTORY plant/velocity/include/ DESTINATION include)
install(DIRECTORY plant/velocity_group/include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(ylr1d_algorithm_sim)
ament_export_targets(ylr1d_algorithm_sim HAS_LIBRARY_TARGET)

# ── 仿真控制器组件库（ROS2 壳：订阅/发布 + 固定线程框架调用算法核心） ──
add_library(ylr1d_algorithm_components SHARED
  node/src/sim_controller.cpp
)
target_include_directories(ylr1d_algorithm_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/node/include>)
target_link_libraries(ylr1d_algorithm_components ylr1d_algorithm_sim)
ament_target_dependencies(ylr1d_algorithm_components
  "rclcpp" "rclcpp_components" "std_msgs" "sensor_msgs")
target_compile_features(ylr1d_algorithm_components PUBLIC cxx_std_17)

rclcpp_components_register_node(ylr1d_algorithm_components
  PLUGIN "ylr1d_algorithm_sim::PositionSimController"
  EXECUTABLE sim_controller_position)
rclcpp_components_register_node(ylr1d_algorithm_components
  PLUGIN "ylr1d_algorithm_sim::VelocitySimController"
  EXECUTABLE sim_controller_velocity)

install(TARGETS ylr1d_algorithm_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY node/include/ DESTINATION include)
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
