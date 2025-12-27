// #include <Eigen/Geometry>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace uav_controllers
{

namespace flight_mode
{

class AttitudeFlightMode : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      // orientation
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

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    attitude_controller_name =
      get_node()->declare_parameter<std::string>("attitude_controller_name");

    mixer_name = get_node()->declare_parameter<std::string>("mixer");

    rpyt_channels = get_node()->declare_parameter<std::vector<int64_t>>("rpyt_channels");

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

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    constexpr double max = 45 * M_PI / 180;
    const double r = scale_rpy(state_interfaces_[0]) * max;
    const double p = scale_rpy(state_interfaces_[1]) * max;
    const double y = -1 * scale_rpy(state_interfaces_[2]);
    const double t = scale_t(state_interfaces_[3]);

    std::cout << "AttitudeFlightMode: r=" << r << ", p=" << p << ", y=" << y << ", t=" << t
              << std::endl;

    bool status_ok = true;

    // Eigen::Quaterniond q_roll(Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()));
    // Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()));
    // Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()));

    // // XYZ convention: Apply roll, then pitch, then yaw
    // Eigen::Quaterniond q_xyz = q_yaw * q_pitch * q_roll;

    // std::cout << "AttitudeFlightMode: q=" << q_xyz.coeffs().transpose() << std::endl;

    // status_ok &= command_interfaces_[0].set_value(0.0);
    // status_ok &= command_interfaces_[1].set_value(0.0);
    // status_ok &= command_interfaces_[2].set_value(0.0);
    // status_ok &= command_interfaces_[3].set_value(1.0);

    status_ok &= command_interfaces_[0].set_value(r);
    status_ok &= command_interfaces_[1].set_value(p);
    status_ok &= command_interfaces_[2].set_value(y);
    status_ok &= command_interfaces_[3].set_value(t / 2);
    // status_ok &= command_interfaces_[4].set_value(0.25);
    // status_ok &= command_interfaces_[4].set_value(0.3);

    if (status_ok)
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      return controller_interface::return_type::ERROR;
    }
  }

private:
  static double scale_rpy(const hardware_interface::LoanedStateInterface & rc_state)
  {
    // scale to [-1, 1], use centred (0) by default
    return (rc_state.get_optional().value_or(1500) - rc_min) / (rc_max - rc_min) * 2 - 1;
  }

  static double scale_t(const hardware_interface::LoanedStateInterface & rc_state)
  {
    // scale to [0, 1], use minimum (0) by default
    return (rc_state.get_optional().value_or(1000) - rc_min) / (rc_max - rc_min);
  }

  std::string attitude_controller_name;
  std::string mixer_name;
  std::vector<int64_t> rpyt_channels;
  static constexpr uint16_t rc_min = 1000;
  static constexpr uint16_t rc_max = 2000;
};

}  // namespace flight_mode

}  // namespace uav_controllers

PLUGINLIB_EXPORT_CLASS(
  uav_controllers::flight_mode::AttitudeFlightMode, controller_interface::ControllerInterface)
