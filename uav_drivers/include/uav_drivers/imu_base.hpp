#pragma once
#include <urdf/model.h>
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

    if (!init_driver())
    {
      return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
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

    assert(imu.orientation.has_value() && std::abs(imu.orientation->norm() - 1) < 1e-6);

    imu.linear_acceleration *= 9.80665;              // g     -> m/s^2
    imu.angular_velocity *= std::numbers::pi / 180;  // deg/s -> rad/s
    *imu.magnetic_field *= 1e6;                      // μT    -> T

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
  std::string sensor_name;
  urdf::Model model;
  std::optional<Eigen::Quaternionf> q_bi;
};

}  // namespace imu

}  // namespace uav::drivers
