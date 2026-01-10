add_library(imu_mpu9250 SHARED imu_mpu9250.cpp)
target_link_libraries(imu_mpu9250 PUBLIC
  imu_base
  i2c
)

install(TARGETS imu_mpu9250
  EXPORT ${PROJECT_NAME}Targets
)
