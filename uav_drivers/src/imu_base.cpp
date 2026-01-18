#include <uav_drivers/imu_base.hpp>

namespace uav::drivers
{

namespace imu
{

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn IMUInterfaceBase::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (const CallbackReturn bcr = hardware_interface::SensorInterface::on_init(params);
      bcr != CallbackReturn::SUCCESS)
  {
    return bcr;
  }

  const std::string sensor_name_param = "sensor_name";

  try
  {
    sensor_name = params.hardware_info.hardware_parameters.at(sensor_name_param);
  }
  catch (std::out_of_range &)
  {
    RCLCPP_FATAL_STREAM(get_logger(), "parameter '" << sensor_name_param << "' not set");
    return CallbackReturn::ERROR;
  }

  // check if sensor exist
  const bool has_sensor =
    params.hardware_info.sensors.cend() !=
    std::find_if(
      params.hardware_info.sensors.cbegin(), params.hardware_info.sensors.cend(),
      [this](const hardware_interface::ComponentInfo & info) { return info.name == sensor_name; });

  if (!has_sensor)
  {
    RCLCPP_FATAL_STREAM(get_logger(), "missing sensor '" << sensor_name << "'");
    return CallbackReturn::ERROR;
  }

  if (!init_driver())
  {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMUInterfaceBase::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // read new IMU data from device
  if (!read_imu_data())
  {
    RCLCPP_ERROR_STREAM(get_logger(), "reading IMU data failed");
  }

  imu.linear_acceleration *= g0;    // g     -> m/s^2
  imu.angular_velocity *= deg2rad;  // deg/s -> rad/s
  if (imu.magnetic_field.has_value())
  {
    (*imu.magnetic_field) *= 1e-6;  // μT    -> T
  }

  set_state<double>(sensor_name + "/angular_velocity.x", imu.angular_velocity.x());
  set_state<double>(sensor_name + "/angular_velocity.y", imu.angular_velocity.y());
  set_state<double>(sensor_name + "/angular_velocity.z", imu.angular_velocity.z());

  set_state<double>(sensor_name + "/linear_acceleration.x", imu.linear_acceleration.x());
  set_state<double>(sensor_name + "/linear_acceleration.y", imu.linear_acceleration.y());
  set_state<double>(sensor_name + "/linear_acceleration.z", imu.linear_acceleration.z());

  if (imu.magnetic_field.has_value())
  {
    set_state<double>(sensor_name + "/magnetic_field.x", imu.magnetic_field->x());
    set_state<double>(sensor_name + "/magnetic_field.y", imu.magnetic_field->y());
    set_state<double>(sensor_name + "/magnetic_field.z", imu.magnetic_field->z());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace imu

}  // namespace uav::drivers
