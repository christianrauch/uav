#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <uav_drivers/imu_base.hpp>

namespace uav::drivers
{

namespace imu
{

class MPU9250 : public IMUInterfaceBase
{
private:
  bool init_driver() override final { return true; }
  bool read_imu_data() override final { return true; }
};

}  // namespace imu

}  // namespace uav::drivers

PLUGINLIB_EXPORT_CLASS(uav::drivers::imu::MPU9250, hardware_interface::SensorInterface)
