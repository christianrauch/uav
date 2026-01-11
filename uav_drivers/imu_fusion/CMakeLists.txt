include(FetchContent)

FetchContent_Declare(
  madgwick_fusion
  GIT_REPOSITORY https://github.com/xioTechnologies/Fusion.git
  GIT_TAG        "v1.2.11"
)
FetchContent_Populate(madgwick_fusion)

add_library(madgwick SHARED
  ${madgwick_fusion_SOURCE_DIR}/Fusion/FusionAhrs.c
  ${madgwick_fusion_SOURCE_DIR}/Fusion/FusionOffset.c
  ${madgwick_fusion_SOURCE_DIR}/Fusion/FusionCompass.c
)
target_link_libraries(madgwick PRIVATE m)
target_include_directories(madgwick PUBLIC
  $<BUILD_INTERFACE:${madgwick_fusion_SOURCE_DIR}/Fusion/>
)
target_compile_features(madgwick PUBLIC cxx_std_20)

install(TARGETS madgwick
  DESTINATION lib
)
