cmake_minimum_required(VERSION 3.8)
project(uav_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_library(proxy_imu SHARED src/proxy_imu.cpp)
target_link_libraries(proxy_imu PUBLIC
  hardware_interface::hardware_interface
  ${sensor_msgs_TARGETS}
)

add_library(proxy_actuator SHARED src/proxy_actuator.cpp)
target_link_libraries(proxy_actuator PUBLIC
  hardware_interface::hardware_interface
  realtime_tools::realtime_tools
  ${mavros_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(hardware_interface ${PROJECT_NAME}_plugin.xml)

install(TARGETS proxy_imu proxy_actuator
  EXPORT ${PROJECT_NAME}Targets
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hardware_interface pluginlib)

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
  # disable xmllint
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
