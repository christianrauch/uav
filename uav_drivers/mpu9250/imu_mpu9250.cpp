#include <pluginlib/class_list_macros.hpp>
#include <uav_drivers/imu_base.hpp>

#include <driver_mpu9250_basic.h>

namespace uav::drivers
{

namespace imu
{

class MPU9250 : public IMUInterfaceBase
{
private:
  bool init_driver() override final
  {
    if (mpu9250_basic_init(MPU9250_INTERFACE_IIC, MPU9250_ADDRESS_AD0_LOW) != 0)
    {
      (void)mpu9250_basic_deinit();
      return false;
    }

    return true;
  }

  bool read_imu_data() override final
  {
    if (
      mpu9250_basic_read(
        imu.linear_acceleration.data(), imu.angular_velocity.data(), imu.magnetic_field->data()) !=
      0)
    {
      (void)mpu9250_basic_deinit();
      return false;
    }

    if (mpu9250_basic_read_temperature(&(*imu.temperature)) != 0)
    {
      (void)mpu9250_basic_deinit();
      return false;
    }

    return true;
  }
};

}  // namespace imu

}  // namespace uav::drivers

PLUGINLIB_EXPORT_CLASS(uav::drivers::imu::MPU9250, hardware_interface::SensorInterface)
