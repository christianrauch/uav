cmake_minimum_required(VERSION 3.8)
project(uav_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(control_toolbox REQUIRED)

add_library(control_allocation SHARED
  src/control_allocation_simple.cpp
  src/control_allocation_wrench.cpp
)
target_link_libraries(control_allocation PUBLIC
  controller_interface::controller_interface
  ${geometry_msgs_TARGETS}
  realtime_tools::realtime_tools
  Eigen3::Eigen
  urdfdom_headers::urdfdom_headers
)
target_compile_definitions(control_allocation PRIVATE EIGEN_INITIALIZE_MATRICES_BY_NAN)

add_library(controller SHARED
  src/controller_angular_velocity_pid.cpp
)
target_link_libraries(controller PUBLIC
  controller_interface::controller_interface
  ${geometry_msgs_TARGETS}
  realtime_tools::realtime_tools
  control_toolbox::control_toolbox
)

pluginlib_export_plugin_description_file(controller_interface uav_controllers_plugin.xml)

install(TARGETS control_allocation controller
  EXPORT ${PROJECT_NAME}Targets
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface geometry_msgs realtime_tools Eigen3 urdfdom_headers)

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
