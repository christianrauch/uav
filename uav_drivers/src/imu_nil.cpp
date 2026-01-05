#include <uav_drivers/imu_base.hpp>

class Nil : public uav::drivers::imu::IMUInterfaceBase
{
private:
  bool init_driver() override final { return true; }
  bool read_imu_data() override final { return true; }
};
