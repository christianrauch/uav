// base on the "imu_sensor_broadcaster" package (Apache License, Version 2.0) by Victor Lopez
// https://github.com/ros-controls/ros2_controllers/tree/4.34.0/imu_sensor_broadcaster

#include <cmath>
#include <controller_interface/controller_interface.hpp>
#include <mavros_msgs/msg/rc_out.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>

namespace rc_broadcaster
{

class RCBroadcaster : public controller_interface::ControllerInterface
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
  using StatePublisher = realtime_tools::RealtimePublisher<mavros_msgs::msg::RCOut>;
  rclcpp::Publisher<mavros_msgs::msg::RCOut>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
  mavros_msgs::msg::RCOut state_message_;
};

controller_interface::CallbackReturn RCBroadcaster::on_init() { return CallbackReturn::SUCCESS; }

controller_interface::CallbackReturn RCBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    sensor_state_publisher_ = get_node()->create_publisher<mavros_msgs::msg::RCOut>(
      "~/rc_out", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL_STREAM(
      get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage with message :" << e.what());
    return CallbackReturn::ERROR;
  }

  state_message_.header.frame_id =
    get_node()->declare_parameter<std::string>("frame_id", std::string{});
  state_message_.channels.resize(8);

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RCBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration RCBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= 8; ++i)
  {
    state_interfaces_config.names.push_back("rc/channel/" + std::to_string(i));
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn RCBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RCBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type RCBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  for (const auto & interface : state_interfaces_)
  {
    for (size_t i = 1; i <= 8; ++i)
    {
      if (interface.get_name() == "rc/channel/" + std::to_string(i))
      {
        if (!std::isnan(interface.get_value()))
        {
          state_message_.channels[i - 1] = static_cast<uint16_t>(interface.get_value());
        }
      }
    }
  }

  if (realtime_publisher_)
  {
    state_message_.header.stamp = time;
    realtime_publisher_->try_publish(state_message_);
  }

  return controller_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(rc_broadcaster::RCBroadcaster, controller_interface::ControllerInterface)

}  // namespace rc_broadcaster
