// base on the "imu_sensor_broadcaster" package (Apache License, Version 2.0) by Victor Lopez
// https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster

#include <Eigen/Dense>
#include <controller_interface/chainable_controller_interface.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <semantic_components/magnetic_field_sensor.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <vector>

namespace magnetometer_broadcaster
{

class MagnetometerBroadcaster : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

protected:
  // std::shared_ptr<ParamListener> param_listener_;
  // Params params_;
  // Eigen::Quaterniond r_;

  std::unique_ptr<semantic_components::MagneticFieldSensor> magnetic_field_sensor_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
  sensor_msgs::msg::MagneticField state_message_;
};

controller_interface::CallbackReturn MagnetometerBroadcaster::on_init()
{
  // try
  // {
  //   param_listener_ = std::make_shared<ParamListener>(get_node());
  // }
  // catch (const std::exception & e)
  // {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
  //     e.what());
  //   return CallbackReturn::ERROR;
  // }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // try
  // {
  //   params_ = param_listener_->get_params();
  //   r_ = quat_from_euler(
  //     params_.rotation_offset.roll, params_.rotation_offset.pitch, params_.rotation_offset.yaw);
  //   r_.normalize();
  // }
  // catch (const std::exception & e)
  // {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(), "Exception thrown during config stage with message: %s \n",
  //     e.what());
  //   return CallbackReturn::ERROR;
  // }

  // magnetic_field_sensor_ =
  //   std::make_unique<semantic_components::MagneticFieldSensor>(params_.sensor_name);
  try
  {
    sensor_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::MagneticField>(
      "~/magnetic_field", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  // state_message_.header.frame_id = params_.frame_id;
  // convert double vector to fixed-size array in the message
  for (size_t i = 0; i < 9; ++i)
  {
    // state_message_.orientation_covariance[i] = params_.static_covariance_orientation[i];
    // state_message_.angular_velocity_covariance[i] =
    // params_.static_covariance_angular_velocity[i];
    // state_message_.magnetic_field_covariance[i] =
    // params_.static_covariance_linear_acceleration[i];
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MagnetometerBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MagnetometerBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = magnetic_field_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  magnetic_field_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  magnetic_field_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type MagnetometerBroadcaster::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::MagneticField input_imu{state_message_};
  magnetic_field_sensor_->get_values_as_message(input_imu);

  if (realtime_publisher_)
  {
    state_message_.header.stamp = time;
    realtime_publisher_->try_publish(state_message_);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type MagnetometerBroadcaster::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
MagnetometerBroadcaster::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported_state_interfaces;

  std::string export_prefix = get_node()->get_name();
  // if (!params_.sensor_name.empty())
  // {
  //   // Update the prefix and get the proper IMU sensor naming
  //   export_prefix = export_prefix + "/" + params_.sensor_name;
  // }

  exported_state_interfaces.emplace_back(hardware_interface::StateInterface(
    export_prefix, "magnetic_field.x", &state_message_.magnetic_field.x));
  exported_state_interfaces.emplace_back(hardware_interface::StateInterface(
    export_prefix, "magnetic_field.y", &state_message_.magnetic_field.y));
  exported_state_interfaces.emplace_back(hardware_interface::StateInterface(
    export_prefix, "magnetic_field.z", &state_message_.magnetic_field.z));

  return exported_state_interfaces;
}

PLUGINLIB_EXPORT_CLASS(
  magnetometer_broadcaster::MagnetometerBroadcaster,
  controller_interface::ChainableControllerInterface)

}  // namespace magnetometer_broadcaster
