#pragma once
#include <Eigen/Geometry>
#include <hardware_interface/sensor_interface.hpp>

namespace uav::drivers
{

namespace imu
{

class IMUInterfaceBase : public hardware_interface::SensorInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  struct
  {
    Eigen::Vector3f angular_velocity = {0, 0, 0};
    Eigen::Vector3f linear_acceleration = {0, 0, 0};
    std::optional<Eigen::Vector3f> magnetic_field;
    std::optional<float> temperature;
  } imu;

  virtual bool read_imu_data() = 0;

  virtual bool init_driver() = 0;

private:
  static constexpr float g0 = 9.80665;                      // m/s^2
  static constexpr float deg2rad = std::numbers::pi / 180;  // rad/s per deg/s

  std::string sensor_name;
  std::optional<Eigen::Vector3f> calib_gyroscope;
  std::optional<Eigen::Affine3f> calib_accelerometer;
  std::optional<Eigen::Affine3f> calib_magnetometer;
};

}  // namespace imu

}  // namespace uav::drivers
