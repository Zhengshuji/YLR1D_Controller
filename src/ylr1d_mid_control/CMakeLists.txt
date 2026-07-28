cmake_minimum_required(VERSION 3.5)
project(ylr1d_mid_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
