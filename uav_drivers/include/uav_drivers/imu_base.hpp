#pragma once
#include <Fusion.h>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <hardware_interface/sensor_interface.hpp>

namespace YAML
{
template <>
struct convert<Eigen::Matrix4f>
{
  static bool decode(const Node & node, Eigen::Matrix4f & mat)
  {
    if (!node.IsSequence() || node.size() != 16)
    {
      return false;
    }

    // convert row-major 16-element sequence to column-major 4x4 matrix
    for (size_t i = 0; i < 16; ++i)
    {
      mat(i / 4, i % 4) = node[i].as<float>();
    }
    return true;
  }
};

template <>
struct convert<Eigen::Vector3f>
{
  static bool decode(const Node & node, Eigen::Vector3f & vec)
  {
    if (!node.IsSequence() || node.size() != 3)
    {
      return false;
    }

    for (size_t i = 0; i < 3; ++i)
    {
      vec(i) = node[i].as<float>();
    }
    return true;
  }
};
}  // namespace YAML

namespace uav::drivers
{

namespace imu
{

class IMUInterfaceBase : public hardware_interface::SensorInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
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
        [this](const hardware_interface::ComponentInfo & info)
        { return info.name == sensor_name; });

    if (!has_sensor)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "missing sensor '" << sensor_name << "'");
      return CallbackReturn::ERROR;
    }

    // parse the robot description
    model.initString(params.hardware_info.original_xml);

    const std::string & root = model.getRoot()->name;

    // get rotation between base link and IMU link
    if (const urdf::LinkConstSharedPtr imu_link = model.getLink(sensor_name))
    {
      if (imu_link->getParent()->name != root)
      {
        RCLCPP_FATAL_STREAM(
          get_node()->get_logger(),
          "IMU link ('" << imu_link->name << "') must be be child of root ('" << root << "')!");
        return CallbackReturn::ERROR;
      }

      const urdf::JointConstSharedPtr joint = imu_link->parent_joint;
      if (joint->type != urdf::Joint::FIXED)
      {
        RCLCPP_FATAL_STREAM(
          get_node()->get_logger(), "IMU joint ('" << joint->name << "') must be of type 'FIXED'!");
        return CallbackReturn::ERROR;
      }

      const urdf::Rotation & q = joint->parent_to_joint_origin_transform.rotation;
      q_bi = {
        static_cast<float>(q.w),
        static_cast<float>(q.x),
        static_cast<float>(q.y),
        static_cast<float>(q.z),
      };
    }

    // read calibration parameters
    const std::string calibration_file_parameter_name = "calibration_file";
    if (params.hardware_info.hardware_parameters.contains(calibration_file_parameter_name))
    {
      const YAML::Node calibration = YAML::LoadFile(
        params.hardware_info.hardware_parameters.at(calibration_file_parameter_name));

      if (calibration["gyroscope"])
      {
        if (!calibration["gyroscope"].IsSequence() || calibration["gyroscope"].size() != 3)
        {
          RCLCPP_FATAL_STREAM(
            get_logger(), "gyroscope calibration must contain a sequence of 3 values");
          return CallbackReturn::ERROR;
        }

        try
        {
          calib_gyroscope = calibration["gyroscope"].as<Eigen::Vector3f>();
        }
        catch (const YAML::BadConversion & e)
        {
          RCLCPP_FATAL_STREAM(
            get_logger(), "Failed to parse gyroscope calibration vector: " << e.what());
          return CallbackReturn::ERROR;
        }
      }

      if (calibration["accelerometer"])
      {
        if (!calibration["accelerometer"].IsSequence() || calibration["accelerometer"].size() != 16)
        {
          RCLCPP_FATAL_STREAM(
            get_logger(), "accelerometer calibration must contain a sequence of 16 values");
          return CallbackReturn::ERROR;
        }

        try
        {
          calib_accelerometer = calibration["accelerometer"].as<Eigen::Matrix4f>();
        }
        catch (const YAML::BadConversion & e)
        {
          RCLCPP_FATAL_STREAM(
            get_logger(), "Failed to parse accelerometer calibration matrix: " << e.what());
          return CallbackReturn::ERROR;
        }
      }

      if (calibration["magnetometer"])
      {
        if (!calibration["magnetometer"].IsSequence() || calibration["magnetometer"].size() != 16)
        {
          RCLCPP_FATAL_STREAM(
            get_logger(), "magnetometer calibration must contain a sequence of 16 values");
          return CallbackReturn::ERROR;
        }

        try
        {
          calib_magnetometer = calibration["magnetometer"].as<Eigen::Matrix4f>();
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

    FusionAhrsInitialise(&fusion);

    if (!init_driver())
    {
      return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    // reset IMU data
    imu = {};

    // read new IMU data from device
    if (!read_imu_data())
    {
      return hardware_interface::return_type::ERROR;
    }

    // apply sensor rotation
    if (q_bi.has_value())
    {
      // rotate data from IMU frame to base frame
      imu.angular_velocity = (*q_bi) * imu.angular_velocity;
      imu.linear_acceleration = (*q_bi) * imu.linear_acceleration;

      if (imu.magnetic_field.has_value())
      {
        *imu.magnetic_field = (*q_bi) * (*imu.magnetic_field);
      }

      if (imu.orientation.has_value())
      {
        imu.orientation = ((*q_bi) * (*imu.orientation) * q_bi->conjugate()).normalized();
      }
    }

    // apply calibration rotate data in SI units
    toSI();
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
    fromSI();

    if (!imu.orientation.has_value())
    {
      // fuse data and compute orientation
      if (imu.magnetic_field.has_value())
      {
        FusionAhrsUpdate(
          &fusion, *reinterpret_cast<FusionVector *>(imu.angular_velocity.data()),
          *reinterpret_cast<FusionVector *>(imu.linear_acceleration.data()),
          *reinterpret_cast<FusionVector *>(imu.magnetic_field->data()), period.seconds());
      }
      else
      {
        FusionAhrsUpdateNoMagnetometer(
          &fusion, *reinterpret_cast<FusionVector *>(imu.angular_velocity.data()),
          *reinterpret_cast<FusionVector *>(imu.linear_acceleration.data()), period.seconds());
      }

      const FusionQuaternion q = FusionAhrsGetQuaternion(&fusion);
      imu.orientation = {q.element.w, q.element.x, q.element.y, q.element.z};
      imu.orientation->normalize();
    }

    assert(imu.orientation.has_value() && std::abs(imu.orientation->norm() - 1) < 1e-6);

    toSI();

    set_state<double>(sensor_name + "/orientation.x", imu.orientation->x());
    set_state<double>(sensor_name + "/orientation.y", imu.orientation->y());
    set_state<double>(sensor_name + "/orientation.z", imu.orientation->z());
    set_state<double>(sensor_name + "/orientation.w", imu.orientation->w());

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

protected:
  struct
  {
    Eigen::Vector3f angular_velocity = {0, 0, 0};
    Eigen::Vector3f linear_acceleration = {0, 0, 0};
    std::optional<Eigen::Vector3f> magnetic_field;
    std::optional<float> temperature;
    std::optional<Eigen::Quaternionf> orientation;
  } imu;

  virtual bool read_imu_data() = 0;

  virtual bool init_driver() = 0;

private:
  static constexpr float g0 = 9.80665;                      // m/s^2
  static constexpr float deg2rad = std::numbers::pi / 180;  // rad/s per deg/s

  void toSI()
  {
    // convert to SI units
    imu.linear_acceleration *= g0;    // g     -> m/s^2
    imu.angular_velocity *= deg2rad;  // deg/s -> rad/s
    if (imu.magnetic_field.has_value())
    {
      (*imu.magnetic_field) *= 1e-6;  // μT    -> T
    }
  }

  void fromSI()
  {
    // convert from SI units
    imu.linear_acceleration /= g0;    // m/s^2 -> g
    imu.angular_velocity /= deg2rad;  // rad/s -> deg/s
    if (imu.magnetic_field.has_value())
    {
      (*imu.magnetic_field) /= 1e-6;  // T    -> μT
    }
  }

  std::string sensor_name;
  urdf::Model model;
  std::optional<Eigen::Quaternionf> q_bi;
  FusionAhrs fusion;
  std::optional<Eigen::Vector3f> calib_gyroscope;
  std::optional<Eigen::Affine3f> calib_accelerometer;
  std::optional<Eigen::Affine3f> calib_magnetometer;
};

}  // namespace imu

}  // namespace uav::drivers
