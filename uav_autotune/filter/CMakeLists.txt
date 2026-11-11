find_package(Eigen3 REQUIRED NO_MODULE)

add_library(LowPassFilter2p SHARED LowPassFilter2p.cpp)
target_link_libraries (LowPassFilter2p
  PUBLIC
    Eigen3::Eigen
)

target_compile_definitions(LowPassFilter2p
  PUBLIC
    EIGEN_INITIALIZE_MATRICES_BY_ZERO
)

add_library(AlphaFilter SHARED AlphaFilter.cpp)
target_link_libraries (AlphaFilter
  PUBLIC
    Eigen3::Eigen
)

target_compile_definitions(AlphaFilter
  PUBLIC
    EIGEN_INITIALIZE_MATRICES_BY_ZERO
)


if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  include(GoogleTest)

  add_executable(LowPassFilter2pVector3fTest LowPassFilter2pVector3fTest.cpp)
  target_link_libraries (LowPassFilter2pVector3fTest
    PRIVATE
      LowPassFilter2p
      GTest::gtest_main
  )

  add_executable(AlphaFilterTest AlphaFilterTest.cpp)
  target_link_libraries (AlphaFilterTest
    PRIVATE
      AlphaFilter
      GTest::gtest_main
  )
endif()
