#include <Eigen/Geometry>
#include <control_msgs/msg/pid_state.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <uav_controllers/controller_attitude_pid_parameters.hpp>
#include "utils.hpp"

namespace uav::controllers
{

namespace attitude
{

class AttitudePID : public controller_interface::ChainableControllerInterface
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
        // orientation [quat]
        sensor_name + "/orientation.x",
        sensor_name + "/orientation.y",
        sensor_name + "/orientation.z",
        sensor_name + "/orientation.w",
        // angular velocity [rad/s]
        sensor_name + "/angular_velocity.x",
        sensor_name + "/angular_velocity.y",
        sensor_name + "/angular_velocity.z",
      }};
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list()
    override
  {
    std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces;

    for (const std::string_view & reference_interface_name : reference_interface_names)
    {
      reference_interfaces.push_back(
        std::make_shared<hardware_interface::CommandInterface>(
          get_node()->get_name(), std::string{reference_interface_name}));
    }

    return reference_interfaces;
  }

  controller_interface::CallbackReturn on_init() override
  {
    param_listener = std::make_shared<ParamListener>(get_node());

    param_listener->setUserCallback(
      [this](const Params & params)
      {
        Gains gains;
        gains.roll_p = params.gains.roll.p;
        gains.roll_i = params.gains.roll.i;
        gains.roll_d = params.gains.roll.d;
        gains.pitch_p = params.gains.pitch.p;
        gains.pitch_i = params.gains.pitch.i;
        gains.pitch_d = params.gains.pitch.d;
        gains.yaw_p = params.gains.yaw_rate.p;

        gains_box.set(gains);
      });

    sub_reference = get_node()->create_subscription<geometry_msgs::msg::Vector3>(
      "~/reference", 1, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) -> void
      { msg_reference.set(*msg); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const Params params = param_listener->get_params();

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

    Gains gains;
    gains.roll_p = params.gains.roll.p;
    gains.roll_i = params.gains.roll.i;
    gains.roll_d = params.gains.roll.d;
    gains.pitch_p = params.gains.pitch.p;
    gains.pitch_i = params.gains.pitch.i;
    gains.pitch_d = params.gains.pitch.d;
    gains.yaw_p = params.gains.yaw_rate.p;

    gains_rt = gains;
    gains_box.set(gains);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    integral_errors.fill(0);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    bool status_ok = true;

    if (const std::optional<geometry_msgs::msg::Vector3> msg = msg_reference.try_get())
    {
      status_ok &= ordered_exported_reference_interfaces_[0]->set_value(msg->x);  // roll [rad]
      status_ok &= ordered_exported_reference_interfaces_[1]->set_value(msg->y);  // pitch [rad]
      status_ok &=
        ordered_exported_reference_interfaces_[2]->set_value(msg->z);  // yaw rate [rad/s]
    }

    return controller_interface::return_type(!status_ok);
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    if (const std::optional<Gains> gains = gains_box.try_get())
    {
      gains_rt = *gains;
    }

    const Gains & gains = gains_rt;

    // orientation [quat]
    const Eigen::Quaterniond q_state{
      state_interfaces_[3].get_optional().value_or(nan),  // w
      state_interfaces_[0].get_optional().value_or(nan),  // x
      state_interfaces_[1].get_optional().value_or(nan),  // y
      state_interfaces_[2].get_optional().value_or(nan),  // z
    };

    // angular velocity [rad/s]
    const double wx = state_interfaces_[4].get_optional().value_or(nan);
    const double wy = state_interfaces_[5].get_optional().value_or(nan);
    const double wz = state_interfaces_[6].get_optional().value_or(nan);

    const Eigen::Matrix3d R_state = q_state.toRotationMatrix();

    // roll: angle between body y-axis and world z-orthogonal plane (clockwise rot. about x-axis)
    const double roll_state = std::asin(R_state(2, 1));
    // pitch: angle between body x-axis and world z-orthogonal plane (clockwise rot. about y-axis)
    const double pitch_state = -std::asin(R_state(2, 0));

    // levelled and no yaw rotation by default
    const std::array<double, n> reference_values = {
      ordered_exported_reference_interfaces_[0]->get_optional().value_or(0),
      ordered_exported_reference_interfaces_[1]->get_optional().value_or(0),
      ordered_exported_reference_interfaces_[2]->get_optional().value_or(0),
    };

    const double roll_err = clamp_deg(reference_values[0]) - roll_state;
    const double pitch_err = clamp_deg(reference_values[1]) - pitch_state;
    const double yaw_rate_err = reference_values[2] - wz;

    if (command_interfaces_.size() != 3)
    {
      return controller_interface::return_type::ERROR;
    }

    // error terms with redundant fields
    msg[0].error = roll_err;
    msg[1].error = pitch_err;
    msg[2].error = yaw_rate_err;
    msg[0].error_dot = -wx;
    msg[1].error_dot = -wy;
    msg[2].error_dot = 0;

    std::array<double, n> cmds;

    // attitude PD-control for roll and pitch
    // The derivative of the position (a) error is related to the velocity (w):
    // de/dt = (a_ref - a_cur) / dt - (a_ref_prev - a_prev) / dt
    //       = (a_ref - a_ref_prev) / dt - (a_cur - a_prev) / dt
    //       = w_ref - w
    // Since the angular velocity is controlled by a downstream controller,
    // we set w_ref = 0, hence de/dt = - w.

    // terms
    const double p_roll = gains.roll_p * roll_err;
    const double p_pitch = gains.pitch_p * pitch_err;
    const double d_roll = gains.roll_d * -wx;
    const double d_pitch = gains.pitch_d * -wy;

    integral_errors[0] += roll_err * period.seconds();
    integral_errors[1] += pitch_err * period.seconds();

    const double i_roll = gains.roll_i * integral_errors[0];
    const double i_pitch = gains.pitch_i * integral_errors[1];

    // attitude PID control
    cmds[0] = p_roll + i_roll + d_roll;
    cmds[1] = p_pitch + i_pitch + d_pitch;

    // yaw rate P-control
    cmds[2] = gains.yaw_p * yaw_rate_err;

    // P gains
    msg[0].p_gain = gains.roll_p;
    msg[1].p_gain = gains.pitch_p;
    msg[2].p_gain = gains.yaw_p;

    // I gains
    msg[0].i_gain = gains.roll_i;
    msg[1].i_gain = gains.pitch_i;
    msg[2].i_gain = 0;

    msg[0].i_term = i_roll;
    msg[1].i_term = i_pitch;
    msg[2].i_term = 0;

    // D gains
    msg[0].d_gain = gains.roll_d;
    msg[1].d_gain = gains.pitch_d;
    msg[2].d_gain = 0;

    bool status_ok = true;

    for (size_t i = 0; i < n; i++)
    {
      status_ok &= command_interfaces_[i].set_value(cmds[i]);
    }

    for (size_t i = 0; i < n; i++)
    {
      msg[i].header.stamp = time;
      msg[i].timestep = period;
      msg[i].output = cmds[i];
      pub_rt_pids[i]->try_publish(msg[i]);
    }

    RCLCPP_DEBUG_STREAM(
      get_node()->get_logger(),
      std::fixed << std::setprecision(2) << std::endl
                 << "roll: " << roll_state << " > " << roll_err << " -> "
                 << command_interfaces_[0].get_optional().value_or(nan) << " [rad/s]" << std::endl
                 << "pitch: " << pitch_state << " > " << pitch_err << " -> "
                 << command_interfaces_[1].get_optional().value_or(nan) << " [rad/s]" << std::endl
                 << "yaw_rate: " << reference_values[2] << " > " << wz << " -> "
                 << command_interfaces_[2].get_optional().value_or(nan) << " [rad/s]");

    return controller_interface::return_type(!status_ok);
  }

private:
  struct Gains
  {
    double roll_p = 0;
    double roll_i = 0;
    double roll_d = 0;
    double pitch_p = 0;
    double pitch_i = 0;
    double pitch_d = 0;
    double yaw_p = 0;
  };

  template <typename T>
  static T clamp_deg(const T & value, const T range_deg = 85)
  {
    const T range = range_deg * std::numbers::pi_v<T> / 180;
    return std::clamp(value, -range, range);
  }

  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();

  static constexpr int8_t n = 3;

  static constexpr std::array<std::string_view, n> command_interfaces = {
    "angular_velocity.x",
    "angular_velocity.y",
    "angular_velocity.z",
  };

  static constexpr std::array<std::string_view, n> reference_interface_names = {
    "roll",
    "pitch",
    "yaw_rate",
  };

  std::string velocity_controller_name;
  std::string sensor_name;
  std::shared_ptr<ParamListener> param_listener;

  realtime_tools::RealtimeThreadSafeBox<Gains> gains_box;
  Gains gains_rt;

  std::array<double, n> integral_errors = {};

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_reference;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Vector3> msg_reference;

  std::array<rclcpp::Publisher<control_msgs::msg::PidState>::SharedPtr, n> pub_pids;
  std::array<realtime_tools::RealtimePublisher<control_msgs::msg::PidState>::UniquePtr, n>
    pub_rt_pids;
  std::array<control_msgs::msg::PidState, n> msg;
};

}  // namespace attitude

}  // namespace uav::controllers

PLUGINLIB_EXPORT_CLASS(
  uav::controllers::attitude::AttitudePID, controller_interface::ChainableControllerInterface)
