cmake_minimum_required(VERSION 3.8)
project(ylr1d_translate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── action 接口 ─────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ChassisMove.action"
  "action/ArmMove.action"
  "action/GripperMove.action"
)

# ── translate_server 节点 ──────────────────────────────
add_executable(translate_server
  src/translate_node.cpp
  src/main.cpp
)
target_include_directories(translate_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(translate_server
  "rclcpp" "rclcpp_action" "std_msgs" "sensor_msgs")
# 链接 rosidl 生成的 action 接口库（提供生成的头文件与类型支持）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(translate_server "${cpp_typesupport_target}")

install(TARGETS translate_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
