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
find_package(ylr1d_translate REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

set(HMI_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ── 控制层 HMI（面向 ylr1d_control_sim，发 /desired_joint_states）──
add_executable(ylr1d_hmi_control
  src/main_control.cpp
  src/panels/hmi_window.cpp
  include/ylr1d_hmi/panels/hmi_window.hpp
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
)
target_include_directories(ylr1d_hmi_sensor PUBLIC ${HMI_INCLUDES})
target_compile_features(ylr1d_hmi_sensor PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_sensor Qt5::Widgets Qt5::Core)
ament_target_dependencies(ylr1d_hmi_sensor "rclcpp" "sensor_msgs" "std_msgs")

# ── 传输层 HMI（面向 ylr1d_translate，发 action goal）────────────
add_executable(ylr1d_hmi_translate
  src/main_translate.cpp
  src/panels/translate_panel.cpp
  include/ylr1d_hmi/panels/translate_panel.hpp
)
target_include_directories(ylr1d_hmi_translate PUBLIC ${HMI_INCLUDES})
target_compile_features(ylr1d_hmi_translate PUBLIC c_std_99 cxx_std_17)
target_link_libraries(ylr1d_hmi_translate Qt5::Widgets Qt5::Core)
ament_target_dependencies(ylr1d_hmi_translate
  "rclcpp" "rclcpp_action" "sensor_msgs" "std_msgs" "ylr1d_translate")

# ── Install all ──────────────────────────────────────────
install(TARGETS ylr1d_hmi_control ylr1d_hmi_sensor ylr1d_hmi_translate
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
