cmake_minimum_required(VERSION 3.8)
project(uav_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Werror=overflow)
  add_link_options("-Wl,-z,defs")
  add_link_options("-Wl,--no-allow-shlib-undefined")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_library(imu_base SHARED src/imu_base.cpp)
target_include_directories(imu_base PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(imu_base PUBLIC
  hardware_interface::hardware_interface
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
)
target_compile_features(imu_base PUBLIC cxx_std_20)

install(TARGETS imu_base
  EXPORT ${PROJECT_NAME}Targets
)

add_subdirectory(mpu9250)
pluginlib_export_plugin_description_file(hardware_interface mpu9250/mpu9250_plugin.xml)

add_subdirectory(pca9685)
pluginlib_export_plugin_description_file(hardware_interface pca9685/pca9685_plugin.xml)

add_subdirectory(sbus)
pluginlib_export_plugin_description_file(hardware_interface sbus/sbus_plugin.xml)

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
