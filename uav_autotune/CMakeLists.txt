cmake_minimum_required(VERSION 3.28)
project(uav_autotune)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_subdirectory(pid_design)
add_subdirectory(filter)
add_subdirectory(system_identification)

set(package_cmake_install_dir lib/cmake/${PROJECT_NAME})
set(CONF_INCLUDE_DIRS "\${uav_autotune_DIR}/../../../include")

configure_file(
  uav_autotuneConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  @ONLY
)

configure_file(
  uav_autotuneConfigVersion.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  @ONLY
)

install(TARGETS
  pid_design
  LowPassFilter2p
  AlphaFilter
  SystemIdentification
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT export_${PROJECT_NAME}
  FILE export_${PROJECT_NAME}.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${package_cmake_install_dir}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${package_cmake_install_dir}
)

install(DIRECTORY
  pid_design
  filter
  system_identification
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)
