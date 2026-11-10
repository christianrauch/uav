cmake_minimum_required(VERSION 3.20)
project(uav_autotune)

set(uav_autotune_MAJOR_VERSION 0)
set(uav_autotune_MINOR_VERSION 0)
set(uav_autotune_PATCH_VERSION 0)
set(uav_autotune_VERSION
  ${uav_autotune_MAJOR_VERSION}.${uav_autotune_MINOR_VERSION}.${uav_autotune_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
