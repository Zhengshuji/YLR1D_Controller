cmake_minimum_required(VERSION 3.8)
project(ylr1d_hmi)

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ylr1d_translate REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

set(HMI_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── 控制层 HMI（面向 ylr1d_control，发 /desired_joint_states）──
add_executable(ylr1d_hmi_control
  src/main_control.cpp
  src/panels/hmi_window.cpp
  include/ylr1d_hmi/panels/hmi_window.hpp
  include/ylr1d_hmi/config/joint_defs.hpp
)
target_include_directories(ylr1d_hmi_control PUBLIC ${HMI_INCLUDES})
target_compile_features(ylr1d_hmi_control PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_control Qt5::Widgets Qt5::Core)
ament_target_dependencies(ylr1d_hmi_control "rclcpp" "sensor_msgs" "std_msgs")

# ── 传感器 HMI（观测 /joint_states + 各传感器话题）────────────
add_executable(ylr1d_hmi_sensor
  src/main_sensor.cpp
  src/panels/sensor_panel.cpp
  include/ylr1d_hmi/panels/sensor_panel.hpp
  include/ylr1d_hmi/config/sensor_topics.hpp
  include/ylr1d_hmi/common/topic_status.hpp
)
target_include_directories(ylr1d_hmi_sensor PUBLIC ${HMI_INCLUDES})
target_compile_features(ylr1d_hmi_sensor PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_sensor Qt5::Widgets Qt5::Core)
ament_target_dependencies(ylr1d_hmi_sensor "rclcpp" "sensor_msgs" "std_msgs")

# ── 传输层 HMI（面向 ylr1d_translate，发 action goal）────────────
add_executable(ylr1d_hmi_translate
  src/main_translate.cpp
  src/panels/translate_panel.cpp
  src/panels/chassis_direction_widget.cpp
  include/ylr1d_hmi/panels/translate_panel.hpp
  include/ylr1d_hmi/config/joint_defs.hpp
  include/ylr1d_hmi/common/action_sender.hpp
)
target_include_directories(ylr1d_hmi_translate PUBLIC ${HMI_INCLUDES})
target_compile_features(ylr1d_hmi_translate PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_translate Qt5::Widgets Qt5::Core)
ament_target_dependencies(ylr1d_hmi_translate
  "rclcpp" "rclcpp_action" "sensor_msgs" "std_msgs" "ylr1d_translate")

# ── 监视 HMI（rviz2 插件，宿主 rviz2 窗口）──────────────
add_library(ylr1d_hmi_monitor_panel SHARED
  src/panels/monitor_panel.cpp
  src/panels/monitor_rviz_panel.cpp
  src/common/sim_control.cpp
  include/ylr1d_hmi/panels/monitor_panel.hpp
  include/ylr1d_hmi/panels/monitor_rviz_panel.hpp
  include/ylr1d_hmi/panels/monitor_nodes.hpp
  include/ylr1d_hmi/common/sim_control.hpp
  include/ylr1d_hmi/common/topic_status.hpp
  include/ylr1d_hmi/config/joint_defs.hpp
  include/ylr1d_hmi/config/sensor_topics.hpp
)
target_include_directories(ylr1d_hmi_monitor_panel PUBLIC ${HMI_INCLUDES})
target_compile_features(ylr1d_hmi_monitor_panel PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_monitor_panel Qt5::Widgets Qt5::Core)
ament_target_dependencies(ylr1d_hmi_monitor_panel
  "rclcpp" "rviz_common" "pluginlib" "sensor_msgs" "std_msgs"
  "rosgraph_msgs" "rcl_interfaces" "controller_manager_msgs" "std_srvs")
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# ── 监视 HMI（独立窗口，无 3D 视图，便于无头验证）────────
add_executable(ylr1d_hmi_monitor
  src/main_monitor.cpp
  src/panels/monitor_panel.cpp
  src/common/sim_control.cpp
  include/ylr1d_hmi/panels/monitor_panel.hpp
  include/ylr1d_hmi/panels/monitor_nodes.hpp
  include/ylr1d_hmi/common/sim_control.hpp
  include/ylr1d_hmi/common/topic_status.hpp
  include/ylr1d_hmi/config/joint_defs.hpp
  include/ylr1d_hmi/config/sensor_topics.hpp
)
target_include_directories(ylr1d_hmi_monitor PUBLIC ${HMI_INCLUDES})
target_compile_features(ylr1d_hmi_monitor PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_monitor Qt5::Widgets Qt5::Core)
ament_target_dependencies(ylr1d_hmi_monitor
  "rclcpp" "sensor_msgs" "std_msgs" "rosgraph_msgs" "rcl_interfaces"
  "controller_manager_msgs" "std_srvs")

# ── Install all ──────────────────────────────────────────
install(TARGETS ylr1d_hmi_control ylr1d_hmi_sensor ylr1d_hmi_translate
        ylr1d_hmi_monitor_panel ylr1d_hmi_monitor
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
