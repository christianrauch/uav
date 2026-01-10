include(FetchContent)

FetchContent_Declare(
  libdriver_mpu9250
  GIT_REPOSITORY  https://github.com/libdriver/mpu9250.git
  GIT_TAG         "v1.0.9"
)
FetchContent_MakeAvailable(libdriver_mpu9250)

set(libmpu9250_src ${libdriver_mpu9250_SOURCE_DIR}/)
set(libmpu9250_pi_src ${libmpu9250_src}/project/raspberrypi4b/)

add_library(mpu9250 STATIC
  ${libmpu9250_src}/src/driver_mpu9250.c
  ${libmpu9250_src}/example/driver_mpu9250_basic.c
  ${libmpu9250_pi_src}/driver/src/raspberrypi4b_driver_mpu9250_interface.c
  ${libmpu9250_pi_src}/interface/src/iic.c
  ${libmpu9250_pi_src}/interface/src/spi.c
  ${libmpu9250_pi_src}/interface/src/gpio.c
  ${libmpu9250_pi_src}/interface/src/mutex.c
)
target_include_directories(mpu9250 PUBLIC
  $<BUILD_INTERFACE:${libmpu9250_src}/src/>
  $<BUILD_INTERFACE:${libmpu9250_src}/interface/>
  $<BUILD_INTERFACE:${libmpu9250_src}/example/>
  $<BUILD_INTERFACE:${libmpu9250_pi_src}/interface/inc/>
)
set_target_properties(mpu9250 PROPERTIES POSITION_INDEPENDENT_CODE TRUE)


add_library(imu_mpu9250 SHARED imu_mpu9250.cpp)
target_link_libraries(imu_mpu9250 PUBLIC
  imu_base
  mpu9250
)

install(TARGETS mpu9250 imu_mpu9250
  EXPORT ${PROJECT_NAME}Targets
)
