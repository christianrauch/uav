add_library(pwm_pca9685 SHARED pwm_pca9685.cpp)
target_link_libraries(pwm_pca9685 PUBLIC
  hardware_interface::hardware_interface
  i2c
)
target_compile_features(pwm_pca9685 PUBLIC cxx_std_20)

install(TARGETS pwm_pca9685
  EXPORT ${PROJECT_NAME}Targets
)
