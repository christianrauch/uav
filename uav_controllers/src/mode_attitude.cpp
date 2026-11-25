#include <Eigen/Geometry>
#include <controller_interface/controller_interface.hpp>
#include <numbers>
#include <pluginlib/class_list_macros.hpp>
#include <uav_controllers/mode_attitude_parameters.hpp>

namespace uav::flight_mode
{

class AttitudeFlightMode : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    std::vector<std::string> interface_names;
    if (control_type == attitude)
    {
      interface_names = {
        // attitude
        control_interface + "/roll",
        control_interface + "/pitch",
        // yaw rate
        angular_velocity_controller_name + "/angular_velocity.z",
      };
    }
    else if (control_type == orientation)
    {
      interface_names = {
        control_interface + "/orientation.x",
        control_interface + "/orientation.y",
        control_interface + "/orientation.z",
        control_interface + "/orientation.w",
      };
    }
    else
    {
      throw std::runtime_error("unsupported control type: '" + std::to_string(control_type) + "'");
    }

    interface_names.push_back(mixer_name + "/thrust");

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

    if (control_type == orientation)
    {
      interface_names.push_back(orientation_sensor_name + "/orientation.x");
      interface_names.push_back(orientation_sensor_name + "/orientation.y");
      interface_names.push_back(orientation_sensor_name + "/orientation.z");
      interface_names.push_back(orientation_sensor_name + "/orientation.w");
    }

    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    param_listener = std::make_shared<ParamListener>(get_node());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const Params params = param_listener->get_params();

    control_type = params.control_type == "attitude" ? attitude : orientation;

    if (!(params.attitude.attitude_controller_name.empty() ||
          params.orientation.controller_name.empty()))
    {
      throw std::runtime_error(
        "exactly one of 'attitude.attitude_controller_name' or 'orientation.controller_name' must "
        "be set");
    }

    if (control_type == attitude)
    {
      if (params.attitude.attitude_controller_name.empty())
      {
        throw std::runtime_error(
          "'attitude' control type requires 'attitude.attitude_controller_name'");
      }
      if (params.attitude.angular_velocity_controller_name.empty())
      {
        throw std::runtime_error(
          "'attitude' control type requires 'attitude.angular_velocity_controller_name'");
      }

      control_interface = params.attitude.attitude_controller_name;
      angular_velocity_controller_name = params.attitude.angular_velocity_controller_name;
    }
    else if (control_type == orientation)
    {
      if (params.orientation.controller_name.empty())
      {
        throw std::runtime_error(
          "'orientation' control type requires 'orientation.controller_name'");
      }
      if (params.orientation.sensor_name.empty())
      {
        throw std::runtime_error("'orientation' control type requires 'orientation.sensor_name'");
      }

      control_interface = params.orientation.controller_name;
      orientation_sensor_name = params.orientation.sensor_name;
    }
    else
    {
      throw std::runtime_error("unsupported control type: '" + params.control_type + "'");
    }

    assert(!control_interface.empty());

    mixer_name = params.mixer;
    rpyt_channels = params.rpyt_channels;

    // maximum roll and pitch in radian
    max_tilt = params.max_tilt * std::numbers::pi / 180;
    // maxiimum taw rate [rad/s]
    max_yaw_rate = params.max_yaw_rate * std::numbers::pi / 180;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    if (control_type == orientation)
    {
      static constexpr double nan = std::numeric_limits<double>::signaling_NaN();

      // initialise the orientation reference
      const std::size_t i = rpyt_channels.size();
      q_reference = {
        state_interfaces_[i + 3].get_optional().value_or(nan),  // w
        state_interfaces_[i + 0].get_optional().value_or(nan),  // x
        state_interfaces_[i + 1].get_optional().value_or(nan),  // y
        state_interfaces_[i + 2].get_optional().value_or(nan),  // z
      };

      q_reference.normalize();
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    const double r = max_tilt * scale_rpy(state_interfaces_[0]);           // [rad]
    const double p = max_tilt * scale_rpy(state_interfaces_[1]);           // [rad]
    const double y = -1 * max_yaw_rate * scale_rpy(state_interfaces_[2]);  // [rad/s]
    const double t = scale_t(state_interfaces_[3]);

    bool status_ok = true;

    if (control_type == attitude)
    {
      status_ok &= command_interfaces_[0].set_value(r);
      status_ok &= command_interfaces_[1].set_value(p);
      status_ok &= command_interfaces_[2].set_value(y);
      status_ok &= command_interfaces_[3].set_value(t);
    }
    else if (control_type == orientation)
    {
      // integrate yaw rate from the RC input
      const double yaw_d = y * period.seconds();
      const Eigen::Quaterniond q_delta_yaw({yaw_d, Eigen::Vector3d::UnitZ()});
      q_reference = q_reference * q_delta_yaw;

      const Eigen::Quaterniond q_pitch({p, Eigen::Vector3d::UnitY()});
      const Eigen::Quaterniond q_roll({r, Eigen::Vector3d::UnitX()});
      const Eigen::Quaterniond q_orientation = q_reference * q_pitch * q_roll;

      status_ok &= command_interfaces_[0].set_value(q_orientation.x());
      status_ok &= command_interfaces_[1].set_value(q_orientation.y());
      status_ok &= command_interfaces_[2].set_value(q_orientation.z());
      status_ok &= command_interfaces_[3].set_value(q_orientation.w());
      status_ok &= command_interfaces_[4].set_value(t);
    }

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

  enum uint8_t
  {
    attitude,
    orientation
  } control_type = attitude;
  std::string angular_velocity_controller_name;
  std::string orientation_sensor_name;
  std::string control_interface;
  std::string mixer_name;
  std::vector<int64_t> rpyt_channels;
  double max_tilt = 0;
  double max_yaw_rate = 0;
  Eigen::Quaterniond q_reference = {};
  std::shared_ptr<ParamListener> param_listener;
  static constexpr double rc_min = 0;
  static constexpr double rc_max = 1;
};

}  // namespace uav::flight_mode

PLUGINLIB_EXPORT_CLASS(
  uav::flight_mode::AttitudeFlightMode, controller_interface::ControllerInterface)
