cmake_minimum_required(VERSION 3.8)
project(mecanum_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mecanum_system_interface SHARED
  src/mecanum_system_interface.cpp
)
target_include_directories(mecanum_system_interface PRIVATE
  include
)
ament_target_dependencies(mecanum_system_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
)
pluginlib_export_plugin_description_file(hardware_interface mecanum_control_plugins.xml)

install(
  TARGETS mecanum_system_interface
  DESTINATION lib
)
install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES mecanum_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(mecanum_system_interface)
ament_package()
