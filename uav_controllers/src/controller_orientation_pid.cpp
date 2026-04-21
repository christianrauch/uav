#include <Eigen/Geometry>
#include <control_msgs/msg/pid_state.hpp>
#include <control_toolbox/pid.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <uav_controllers/controller_orientation_pid_parameters.hpp>
#include "utils.hpp"

namespace uav::controllers
{

namespace angular_position
{

class AngPosPID : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {
        std::format("{}/{}", velocity_controller_name, command_interfaces[0]),
        std::format("{}/{}", velocity_controller_name, command_interfaces[1]),
        std::format("{}/{}", velocity_controller_name, command_interfaces[2]),
      }};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {
        sensor_name + "/orientation.x",
        sensor_name + "/orientation.y",
        sensor_name + "/orientation.z",
        sensor_name + "/orientation.w",
      }};
  }

  controller_interface::CallbackReturn on_init() override
  {
    sub_reference = get_node()->create_subscription<geometry_msgs::msg::Quaternion>(
      "~/reference", 1, [this](const geometry_msgs::msg::Quaternion::SharedPtr msg) -> void
      { msg_reference.set(*msg); });

    param_listener = std::make_shared<ParamListener>(get_node());

    param_listener->setUserCallback([this](const Params & params) { configure_pid(params); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const Params params = param_listener->get_params();

    // set reference interface
    exported_reference_interface_names_.assign(
      reference_interface_names.begin(), reference_interface_names.end());
    reference_interfaces_.resize(exported_reference_interface_names_.size(), nan);

    velocity_controller_name = params.velocity_controller_name;
    sensor_name = params.sensor_name;

    for (size_t i = 0; i < n; i++)
    {
      pub_pids[i] = get_node()->create_publisher<control_msgs::msg::PidState>(
        std::format("~/{}/{}", "pid", interface_to_topic(command_interfaces[i])),
        rclcpp::SystemDefaultsQoS());
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
    tau = params.gains.tau;

    control_toolbox::AntiWindupStrategy antiwindup;
    antiwindup.set_type(params.antiwindup.type);
    antiwindup.i_min = params.antiwindup.i_min;
    antiwindup.i_max = params.antiwindup.i_max;
    antiwindup.tracking_time_constant = params.antiwindup.tracking_time_constant;

    antiwindup.validate();

    const std::unordered_map<std::string, std::array<double, 3>> gains = {
      {"roll", {params.gains.roll.p, params.gains.roll.i, params.gains.roll.d}},
      {"pitch", {params.gains.pitch.p, params.gains.pitch.i, params.gains.pitch.d}},
      {"yaw", {params.gains.yaw.p, params.gains.yaw.i, params.gains.yaw.d}},
    };

    bool all_success = true;

    for (size_t i = 0; i < cmd_order.size(); i++)
    {
      const std::array<double, 3> g = gains.at(cmd_order[i]);

      const bool success = pid_controllers[i].set_gains(
        // gains
        g[0], g[1], g[2],
        // output limits
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
        // anti integral windup settings
        antiwindup);

      if (success)
      {
        const control_toolbox::Pid::Gains gains = pid_controllers[i].get_gains();
        RCLCPP_INFO_STREAM(
          get_node()->get_logger(), "PID for '" << cmd_order[i] << "': p: " << gains.p_gain_
                                                << ", i: " << gains.i_gain_
                                                << ", d: " << gains.d_gain_);
      }
      else
      {
        RCLCPP_ERROR_STREAM(
          get_node()->get_logger(), "Failed to set PID gains for '" << cmd_order[i] << "'");
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
    if (const std::optional<geometry_msgs::msg::Quaternion> msg = msg_reference.try_get())
    {
      reference_interfaces_ = {
        msg->x,
        msg->y,
        msg->z,
        msg->w,
      };
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    const Eigen::Quaterniond q_ref{
      reference_interfaces_[3],  // w
      reference_interfaces_[0],  // x
      reference_interfaces_[1],  // y
      reference_interfaces_[2],  // z
    };

    const Eigen::Quaterniond q_state{
      state_interfaces_[3].get_optional().value_or(nan),  // w
      state_interfaces_[0].get_optional().value_or(nan),  // x
      state_interfaces_[1].get_optional().value_or(nan),  // y
      state_interfaces_[2].get_optional().value_or(nan),  // z
    };

    constexpr double eps = 1e-6;

    if (std::abs(q_state.norm() - 1) > eps)
    {
      RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        std::fixed << std::setprecision(6)
                   << "State quaternion is not normalised (norm: " << q_state.norm() << ")!");
    }

    if (std::abs(q_ref.norm() - 1) > eps)
    {
      RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        std::fixed << std::setprecision(6)
                   << "Reference quaternion is not normalised (norm: " << q_ref.norm() << ")!");
    }

    // error quaternion
    const Eigen::Quaterniond q_err = q_state.normalized().conjugate() * q_ref.normalized();

    // error quaternion with positive scalar part
    const Eigen::Quaterniond q_err_A =
      (q_err.w() < 0) ? Eigen::Quaterniond{-q_err.coeffs()} : q_err;

    RCLCPP_DEBUG_STREAM(
      get_node()->get_logger(), std::fixed << std::setprecision(2)
                                           << "error (x,y,z,w): " << q_ref.coeffs().transpose()
                                           << " <> " << q_state.coeffs().transpose() << " -> "
                                           << q_err_A.coeffs().transpose());

    const Eigen::Vector3d o = 2 / tau * q_err_A.vec();

    assert(n == o.size());

    bool status_ok = true;

    for (Eigen::Index i = 0; i < n; i++)
    {
      const double cmd = pid_controllers[i].compute_command(o[i], period);
      RCLCPP_DEBUG_STREAM(
        get_node()->get_logger(), std::fixed << std::setprecision(2) << cmd_order[i] << ": " << o[i]
                                             << " -> " << cmd << " [rad]");
      status_ok &= command_interfaces_[i].set_value(cmd);

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

    return controller_interface::return_type(!status_ok);
  }

private:
  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();

  static constexpr int8_t n = 3;  // output or control dimension

  static constexpr std::array<std::string, n> cmd_order = {"roll", "pitch", "yaw"};

  static constexpr std::array<std::string_view, n> command_interfaces = {
    "angular_velocity.x",
    "angular_velocity.y",
    "angular_velocity.z",
  };

  static constexpr std::array<std::string_view, 4> reference_interface_names = {
    "orientation.x",
    "orientation.y",
    "orientation.z",
    "orientation.w",
  };

  std::string velocity_controller_name;
  std::string sensor_name;
  double tau;
  std::shared_ptr<ParamListener> param_listener;

  std::array<control_toolbox::Pid, n> pid_controllers;

  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_reference;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Quaternion> msg_reference;

  std::array<rclcpp::Publisher<control_msgs::msg::PidState>::SharedPtr, n> pub_pids;
  std::array<realtime_tools::RealtimePublisher<control_msgs::msg::PidState>::UniquePtr, n>
    pub_rt_pids;
  std::array<control_msgs::msg::PidState, n> msg;
};

}  // namespace angular_position

}  // namespace uav::controllers

PLUGINLIB_EXPORT_CLASS(
  uav::controllers::angular_position::AngPosPID, controller_interface::ChainableControllerInterface)
