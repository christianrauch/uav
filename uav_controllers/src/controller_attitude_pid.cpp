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

  controller_interface::CallbackReturn on_init() override
  {
    param_listener = std::make_shared<ParamListener>(get_node());

    param_listener->setUserCallback(
      [this](const Params & params)
      {
        mutex_param_gains.lock();
        gain_roll_p = params.gains.roll.p;
        gain_roll_d = params.gains.roll.d;
        gain_pitch_p = params.gains.pitch.p;
        gain_pitch_d = params.gains.pitch.d;
        gain_yaw_p = params.gains.yaw_rate.p;
        mutex_param_gains.unlock();
      });

    sub_reference = get_node()->create_subscription<geometry_msgs::msg::Vector3>(
      "~/reference", 1, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) -> void
      { msg_reference.set(*msg); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const Params params = param_listener->get_params();

    exported_reference_interface_names_.assign(
      reference_interface_names.begin(), reference_interface_names.end());
    // levelled and no yaw rotation by default
    reference_interfaces_.resize(exported_reference_interface_names_.size(), 0);

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

    mutex_param_gains.lock();
    gain_roll_p = params.gains.roll.p;
    gain_roll_d = params.gains.roll.d;
    gain_pitch_p = params.gains.pitch.p;
    gain_pitch_d = params.gains.pitch.d;
    gain_yaw_p = params.gains.yaw_rate.p;
    mutex_param_gains.unlock();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (const std::optional<geometry_msgs::msg::Vector3> msg = msg_reference.try_get())
    {
      reference_interfaces_ = {
        msg->x,  // roll [rad]
        msg->y,  // pitch [rad]
        msg->z,  // yaw rate [rad/s]
      };
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
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

    const double roll_err = clamp_deg(reference_interfaces_[0]) - roll_state;
    const double pitch_err = clamp_deg(reference_interfaces_[1]) - pitch_state;
    const double yaw_rate_err = reference_interfaces_[2] - wz;

    if (command_interfaces_.size() != 3)
    {
      return controller_interface::return_type::ERROR;
    }

    // error terms with redundant fields
    msg[0].error = msg[0].p_error = roll_err;
    msg[1].error = msg[1].p_error = pitch_err;
    msg[2].error = msg[2].p_error = yaw_rate_err;
    msg[0].error_dot = msg[0].d_error = wx;
    msg[1].error_dot = msg[1].d_error = wy;
    msg[2].error_dot = msg[2].d_error = 0;
    // PD controller without I term
    msg[0].i_error = msg[1].i_error = msg[2].i_error = 0;

    std::array<double, n> cmds;

    mutex_param_gains.lock();

    // attitude PD-control for roll and pitch
    cmds[0] = gain_roll_p * roll_err + gain_roll_d * wx;
    cmds[1] = gain_pitch_p * pitch_err + gain_pitch_d * wy;
    // yaw rate P-control
    cmds[2] = gain_yaw_p * yaw_rate_err;

    // P gains
    msg[0].p_term = gain_roll_p;
    msg[1].p_term = gain_pitch_p;
    msg[2].p_term = gain_yaw_p;
    // D gains
    msg[0].d_term = gain_roll_d;
    msg[1].d_term = gain_pitch_d;
    msg[2].d_term = 0;
    // no I gains
    msg[0].i_term = msg[1].i_term = msg[2].i_term = 0;

    mutex_param_gains.unlock();

    bool status_ok = true;

    for (size_t i = 0; i < n; i++)
    {
      status_ok &= command_interfaces_[i].set_value(cmds[i]);

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
                 << "yaw_rate: " << reference_interfaces_[2] << " > " << wz << " -> "
                 << command_interfaces_[2].get_optional().value_or(nan) << " [rad/s]");

    return controller_interface::return_type(!status_ok);
  }

private:
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
  std::mutex mutex_param_gains;

  double gain_roll_p = 0;
  double gain_roll_d = 0;
  double gain_pitch_p = 0;
  double gain_pitch_d = 0;
  double gain_yaw_p = 0;

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
