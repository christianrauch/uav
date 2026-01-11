// base on the "imu_sensor_broadcaster" package (Apache License, Version 2.0) by Victor Lopez
// https://github.com/ros-controls/ros2_controllers/tree/4.34.0/imu_sensor_broadcaster

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
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

class MagnetometerBroadcaster : public controller_interface::ControllerInterface
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

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::unique_ptr<semantic_components::MagneticFieldSensor> magnetic_field_sensor_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
  sensor_msgs::msg::MagneticField state_message_;
};

controller_interface::CallbackReturn MagnetometerBroadcaster::on_init()
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MagnetometerBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::string sensor_name = get_node()->declare_parameter<std::string>("sensor_name");
  if (sensor_name.empty())
  {
    RCLCPP_FATAL_STREAM(
      get_node()->get_logger(),
      "Parameter 'sensor_name' is empty. Please specify a sensor name for the magnetometer.");
    return controller_interface::CallbackReturn::ERROR;
  }

  magnetic_field_sensor_ = std::make_unique<semantic_components::MagneticFieldSensor>(sensor_name);
  try
  {
    sensor_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::MagneticField>(
      "~/magnetic_field", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL_STREAM(
      get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage with message :" << e.what());
    return CallbackReturn::ERROR;
  }

  const std::string frame_id =
    get_node()->declare_parameter<std::string>("frame_id", std::string{});
  state_message_.header.frame_id = (frame_id.empty()) ? sensor_name : frame_id;

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

controller_interface::return_type MagnetometerBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  magnetic_field_sensor_->get_values_as_message(state_message_);

  if (realtime_publisher_)
  {
    state_message_.header.stamp = time;
    realtime_publisher_->try_publish(state_message_);
  }

  return controller_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(
  magnetometer_broadcaster::MagnetometerBroadcaster, controller_interface::ControllerInterface)

}  // namespace magnetometer_broadcaster
