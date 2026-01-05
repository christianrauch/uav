#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace uav::flight_mode
{

class AttitudeFlightMode : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      // attitude
      attitude_controller_name + "/roll",
      attitude_controller_name + "/pitch",
      // yaw rate
      attitude_controller_name + "/yaw_rate",
      // thrust
      mixer_name + "/thrust",
    };
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> interface_names;
    for (const int64_t channel_id : rpyt_channels)
    {
      assert(channel_id > 0);
      interface_names.push_back("rc/channel/" + std::to_string(channel_id));
    }
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    attitude_controller_name =
      get_node()->declare_parameter<std::string>("attitude_controller_name");

    mixer_name = get_node()->declare_parameter<std::string>("mixer");

    rpyt_channels = get_node()->declare_parameter<std::vector<int64_t>>("rpyt_channels");

    // maximum roll and pitch in degree
    max_tilt = get_node()->declare_parameter<double>("max_tilt", 60);

    if (rpyt_channels.size() != 4)
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(),
        "Parameter 'rpyt_channels' must contain exactly 4 channel IDs for roll, pitch, yaw, and "
        "throttle.");
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    const double max = max_tilt * std::numbers::pi / 180;
    const double r = scale_rpy(state_interfaces_[0]) * max;
    const double p = scale_rpy(state_interfaces_[1]) * max;
    const double y = -1 * scale_rpy(state_interfaces_[2]);
    const double t = scale_t(state_interfaces_[3]);

    bool status_ok = true;

    status_ok &= command_interfaces_[0].set_value(r);
    status_ok &= command_interfaces_[1].set_value(p);
    status_ok &= command_interfaces_[2].set_value(y);
    status_ok &= command_interfaces_[3].set_value(t / 2);

    return controller_interface::return_type(!status_ok);
  }

private:
  static double scale_rpy(const hardware_interface::LoanedStateInterface & rc_state)
  {
    // scale to [-1, 1], use centred (0) by default
    return (rc_state.get_optional().value_or(0.5) - rc_min) / (rc_max - rc_min) * 2 - 1;
  }

  static double scale_t(const hardware_interface::LoanedStateInterface & rc_state)
  {
    // scale to [0, 1], use minimum (0) by default
    return (rc_state.get_optional().value_or(0) - rc_min) / (rc_max - rc_min);
  }

  std::string attitude_controller_name;
  std::string mixer_name;
  std::vector<int64_t> rpyt_channels;
  double max_tilt;
  static constexpr double rc_min = 0;
  static constexpr double rc_max = 1;
};

}  // namespace uav::flight_mode

PLUGINLIB_EXPORT_CLASS(
  uav::flight_mode::AttitudeFlightMode, controller_interface::ControllerInterface)
