#include <control_msgs/msg/pid_state.hpp>
#include <control_toolbox/pid.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <uav_controllers/controller_angular_velocity_pid_parameters.hpp>
#include "utils.hpp"

namespace uav::controllers
{

namespace angular_velocity
{

class AngVelPID : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      mixer_name + "/" + command_interfaces[0],
      mixer_name + "/" + command_interfaces[1],
      mixer_name + "/" + command_interfaces[2],
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      sensor_name + "/angular_velocity.x",
      sensor_name + "/angular_velocity.y",
      sensor_name + "/angular_velocity.z",
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    param_listener = std::make_shared<ParamListener>(get_node());

    param_listener->setUserCallback([this](const Params & params) { configure_pid(params); });

    sub_reference = get_node()->create_subscription<geometry_msgs::msg::Vector3>(
      "~/reference", 1, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) -> void
      { msg_reference.set(*msg); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const Params params = param_listener->get_params();
    mixer_name = params.mixer;
    command_interfaces = params.command_interfaces;
    sensor_name = params.sensor_name;

    // set reference interface
    exported_reference_interface_names_.assign(
      reference_interface_names.begin(), reference_interface_names.end());
    reference_interfaces_.resize(exported_reference_interface_names_.size(), nan);

    assert(command_interfaces.size() == n);

    for (size_t i = 0; i < n; i++)
    {
      pub_pids[i] = get_node()->create_publisher<control_msgs::msg::PidState>(
        "~/pid/" + interface_to_topic(command_interfaces[i]), rclcpp::SystemDefaultsQoS());
      pub_rt_pids[i] =
        std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::PidState>>(
          pub_pids[i]);
    }

    if (configure_pid(params))
    {
      return controller_interface::CallbackReturn::SUCCESS;
    }
    else
    {
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  bool configure_pid(const Params & params)
  {
    control_toolbox::AntiWindupStrategy antiwindup;
    antiwindup.set_type(params.antiwindup.type);
    antiwindup.i_min = params.antiwindup.i_min;
    antiwindup.i_max = params.antiwindup.i_max;
    antiwindup.tracking_time_constant = params.antiwindup.tracking_time_constant;

    antiwindup.validate();

    bool all_success = true;

    for (size_t i = 0; i < command_interfaces.size(); i++)
    {
      const bool success = pid_controllers[i].set_gains(
        // gains
        params.gains.command_interfaces_map.at(command_interfaces[i]).p,
        params.gains.command_interfaces_map.at(command_interfaces[i]).i,
        params.gains.command_interfaces_map.at(command_interfaces[i]).d,
        // output limits
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
        // anti integral windup settings
        antiwindup);

      if (success)
      {
        const control_toolbox::Pid::Gains gains = pid_controllers[i].get_gains();
        RCLCPP_INFO_STREAM(
          get_node()->get_logger(), "PID for '"
                                      << command_interfaces[i] << "': p: " << gains.p_gain_
                                      << ", i: " << gains.i_gain_ << ", d: " << gains.d_gain_);
      }
      else
      {
        RCLCPP_ERROR_STREAM(
          get_node()->get_logger(),
          "Failed to set PID gains for '" << command_interfaces[i] << "'");
      }

      all_success &= success;
    }

    return all_success;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    for (control_toolbox::Pid & pid_controller : pid_controllers)
    {
      pid_controller.reset();
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (const std::optional<geometry_msgs::msg::Vector3> msg = msg_reference.try_get())
    {
      reference_interfaces_ = {
        msg->x,  // roll
        msg->y,  // pitch
        msg->z,  // yaw
      };
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    // roll, pitch, yaw
    std::array<double, n> cmds;
    for (size_t i = 0; i < pid_controllers.size(); i++)
    {
      const double state = state_interfaces_[i].get_optional().value_or(nan);
      cmds[i] = pid_controllers[i].compute_command(reference_interfaces_[i] - state, period);
      RCLCPP_DEBUG_STREAM(
        get_node()->get_logger(), std::fixed << std::setprecision(2) << command_interfaces[i]
                                             << ": " << reference_interfaces_[i] << " <> " << state
                                             << " -> " << cmds[i] << " [rad/s]");

      msg[i].header.stamp = time;
      msg[i].timestep = period;

      // error terms
      pid_controllers[i].get_current_pid_errors(
        msg[i].error,     // error = target - state
        msg[i].i_error,   // weighted integral error
        msg[i].error_dot  // derivative of error
      );

      // set redundant fields
      msg[i].p_error = msg[i].error;
      msg[i].d_error = msg[i].error_dot;

      // gains
      pid_controllers[i].get_gains(
        msg[i].p_term, msg[i].i_term, msg[i].d_term, msg[i].i_max, msg[i].i_min);

      msg[i].output = pid_controllers[i].get_current_cmd();

      pub_rt_pids[i]->try_publish(msg[i]);
    }

    bool status_ok = true;

    for (size_t i = 0; i < cmds.size(); i++)
    {
      status_ok &= command_interfaces_[i].set_value(cmds[i]);
    }

    return controller_interface::return_type(!status_ok);
  }

private:
  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();

  std::string mixer_name;
  std::vector<std::string> command_interfaces;
  std::string sensor_name;
  std::shared_ptr<ParamListener> param_listener = nullptr;

  static constexpr int8_t n = 3;
  std::array<control_toolbox::Pid, n> pid_controllers;

  static constexpr std::array<std::string_view, 3> reference_interface_names = {
    "angular_velocity.x",
    "angular_velocity.y",
    "angular_velocity.z",
  };

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_reference;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Vector3> msg_reference;

  std::array<rclcpp::Publisher<control_msgs::msg::PidState>::SharedPtr, n> pub_pids;
  std::array<realtime_tools::RealtimePublisher<control_msgs::msg::PidState>::UniquePtr, n>
    pub_rt_pids;
  std::array<control_msgs::msg::PidState, n> msg;
};

}  // namespace angular_velocity

}  // namespace uav::controllers

PLUGINLIB_EXPORT_CLASS(
  uav::controllers::angular_velocity::AngVelPID, controller_interface::ChainableControllerInterface)
