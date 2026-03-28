cmake_minimum_required(VERSION 3.8)
project(uav_mode_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_library(mode_manager SHARED src/mode_manager.cpp)
target_link_libraries(mode_manager PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  ${control_msgs_TARGETS}
  ${controller_manager_msgs_TARGETS}
)
target_compile_features(mode_manager PUBLIC cxx_std_17)

rclcpp_components_register_node(mode_manager PLUGIN "ModeManager" EXECUTABLE "mode_manager_node")


install(TARGETS mode_manager
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  control_msgs
  controller_manager_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # disable uncrustify
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
