add_library(rc_sbus SHARED rc_sbus.cpp)
target_link_libraries(rc_sbus PUBLIC
  hardware_interface::hardware_interface
)
target_compile_features(rc_sbus PUBLIC cxx_std_20)

install(TARGETS rc_sbus
  EXPORT ${PROJECT_NAME}Targets
)
