cmake_minimum_required(VERSION 3.28)
project(uav_autotune_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_autotune REQUIRED)

add_library(uav_autotune_component SHARED src/uav_autotune.cpp)
target_compile_features(uav_autotune_component PUBLIC cxx_std_20)
target_link_libraries(uav_autotune_component PUBLIC
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  std_msgs::std_msgs__rosidl_generator_cpp
  uav_autotune::AlphaFilter
  uav_autotune::SystemIdentification
)

rclcpp_components_register_node(uav_autotune_component
  PLUGIN "UavAutotuneNode"
  EXECUTABLE uav_autotune_node
)

install(TARGETS uav_autotune_component
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components uav_autotune)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
