#include <yaml-cpp/yaml.h>
#include <uav_drivers/imu_base.hpp>

namespace YAML
{
template <>
struct convert<Eigen::Matrix4d>
{
  static bool decode(const Node & node, Eigen::Matrix4d & mat)
  {
    if (!node.IsSequence() || node.size() != 16)
    {
      return false;
    }

    // convert row-major 16-element sequence to column-major 4x4 matrix
    for (size_t i = 0; i < 16; ++i)
    {
      mat(i / 4, i % 4) = node[i].as<double>();
    }
    return true;
  }
};

template <>
struct convert<Eigen::Vector3d>
{
  static bool decode(const Node & node, Eigen::Vector3d & vec)
  {
    if (!node.IsSequence() || node.size() != 3)
    {
      return false;
    }

    for (size_t i = 0; i < 3; ++i)
    {
      vec(i) = node[i].as<double>();
    }
    return true;
  }
};
}  // namespace YAML

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

  // check if sensor exists
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

  // read calibration parameters
  const std::string calibration_file_parameter_name = "calibration_file";
  if (params.hardware_info.hardware_parameters.contains(calibration_file_parameter_name))
  {
    const YAML::Node calibration =
      YAML::LoadFile(params.hardware_info.hardware_parameters.at(calibration_file_parameter_name));

    const YAML::Node & g = calibration["gyroscope"];
    if (g && !g.IsNull())
    {
      if (!g.IsSequence() || g.size() != 3)
      {
        RCLCPP_FATAL_STREAM(
          get_logger(), "gyroscope calibration must contain a sequence of 3 values");
        return CallbackReturn::ERROR;
      }

      try
      {
        calib_gyroscope = g.as<Eigen::Vector3d>();
      }
      catch (const YAML::BadConversion & e)
      {
        RCLCPP_FATAL_STREAM(
          get_logger(), "Failed to parse gyroscope calibration vector: " << e.what());
        return CallbackReturn::ERROR;
      }
    }

    const YAML::Node & a = calibration["accelerometer"];
    if (a && !a.IsNull())
    {
      if (!a.IsSequence() || a.size() != 16)
      {
        RCLCPP_FATAL_STREAM(
          get_logger(), "accelerometer calibration must contain a sequence of 16 values");
        return CallbackReturn::ERROR;
      }

      try
      {
        calib_accelerometer = a.as<Eigen::Matrix4d>();
      }
      catch (const YAML::BadConversion & e)
      {
        RCLCPP_FATAL_STREAM(
          get_logger(), "Failed to parse accelerometer calibration matrix: " << e.what());
        return CallbackReturn::ERROR;
      }
    }

    const YAML::Node & m = calibration["magnetometer"];
    if (m && !m.IsNull())
    {
      if (!m.IsSequence() || m.size() != 16)
      {
        RCLCPP_FATAL_STREAM(
          get_logger(), "magnetometer calibration must contain a sequence of 16 values");
        return CallbackReturn::ERROR;
      }

      try
      {
        calib_magnetometer = m.as<Eigen::Matrix4d>();
      }
      catch (const YAML::BadConversion & e)
      {
        RCLCPP_FATAL_STREAM(
          get_logger(), "Failed to parse magnetometer calibration matrix: " << e.what());
        return CallbackReturn::ERROR;
      }
    }

    if (calib_gyroscope.has_value())
    {
      RCLCPP_INFO_STREAM(
        get_logger(), std::endl
                        << "gyroscope calibration:" << std::endl
                        << (*calib_gyroscope).transpose());
    }

    if (calib_accelerometer.has_value())
    {
      RCLCPP_INFO_STREAM(
        get_logger(), std::endl
                        << "accelerometer calibration:" << std::endl
                        << (*calib_accelerometer).matrix());
    }

    if (calib_magnetometer.has_value())
    {
      RCLCPP_INFO_STREAM(
        get_logger(), std::endl
                        << "magnetometer calibration:" << std::endl
                        << (*calib_magnetometer).matrix());
    }
  }

  if (!init_driver())
  {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
IMUInterfaceBase::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!deinit_driver())
  {
    return CallbackReturn::FAILURE;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IMUInterfaceBase::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // read new IMU data from device
  if (!read_imu_data())
  {
    RCLCPP_ERROR_STREAM(get_logger(), "reading IMU data failed");
  }

  // convert units
  imu.linear_acceleration *= g0;    // g     -> m/s^2
  imu.angular_velocity *= deg2rad;  // deg/s -> rad/s
  if (imu.magnetic_field.has_value())
  {
    (*imu.magnetic_field) *= 1e-6;  // μT    -> T
  }

  // apply calibration
  if (calib_gyroscope.has_value())
  {
    imu.angular_velocity = imu.angular_velocity - (*calib_gyroscope);
  }

  if (calib_accelerometer.has_value())
  {
    imu.linear_acceleration = (*calib_accelerometer) * imu.linear_acceleration;
  }

  if (calib_magnetometer.has_value() && imu.magnetic_field.has_value())
  {
    (*imu.magnetic_field) = (*calib_magnetometer) * (*imu.magnetic_field);
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
