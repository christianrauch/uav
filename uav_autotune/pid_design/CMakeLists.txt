find_package(Eigen3 REQUIRED NO_MODULE)

add_library(pid_design SHARED pid_design.cpp)
target_include_directories(pid_design PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pid_design
  PUBLIC
    Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  include(GoogleTest)

  add_executable(pid_design_test pid_design_test.cpp)
  target_link_libraries (pid_design_test
    PRIVATE
      pid_design
      GTest::gtest_main
  )
endif()
