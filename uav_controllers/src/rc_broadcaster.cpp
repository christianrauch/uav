// base on the "imu_sensor_broadcaster" package (Apache License, Version 2.0) by Victor Lopez
// https://github.com/ros-controls/ros2_controllers/tree/4.34.0/imu_sensor_broadcaster

#include <algorithm>
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

  static const std::string rc_prefix;
  uint8_t nchannels = 0;
};

const std::string RCBroadcaster::rc_prefix = "rc/channel/";

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
  state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
  return state_interfaces_config;
}

controller_interface::CallbackReturn RCBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  nchannels = std::count_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [](const auto & iface) { return iface.get_name().substr(0, rc_prefix.size()) == rc_prefix; });

  state_message_.channels.resize(nchannels);

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
    for (size_t i = 1; i <= nchannels; ++i)
    {
      if (interface.get_name() == rc_prefix + std::to_string(i))
      {
        auto val = interface.get_optional();
        if (val.has_value() && !std::isnan(val.value()))
        {
          state_message_.channels[i - 1] = static_cast<uint16_t>(val.value()*1000);
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
