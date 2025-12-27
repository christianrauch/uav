#include <Eigen/Geometry>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace uav_controllers
{

namespace attitude
{

class AttitudePID : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      mixer_name + "/roll",
      mixer_name + "/pitch",
      mixer_name + "/yaw",
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      // orientation [quat]
      sensor_name + "/orientation.x",
      sensor_name + "/orientation.y",
      sensor_name + "/orientation.z",
      sensor_name + "/orientation.w",
      // angular velocity [rad/s]
      sensor_name + "/angular_velocity.x",
      sensor_name + "/angular_velocity.y",
      sensor_name + "/angular_velocity.z",
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    exported_reference_interface_names_ = {
      "roll",
      "pitch",
      "yaw_rate",
    };
    // levelled and no yaw rotation by default
    reference_interfaces_.resize(exported_reference_interface_names_.size(), 0);

    sub_reference = get_node()->create_subscription<geometry_msgs::msg::Vector3>(
      "~/reference", 1, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) -> void
      { msg_reference.set(*msg); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    mixer_name = get_node()->declare_parameter<std::string>("mixer");

    if (mixer_name.empty())
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(), "Parameter 'mixer' is empty. Please specify a mixer name.");
      return controller_interface::CallbackReturn::ERROR;
    }

    sensor_name = get_node()->declare_parameter<std::string>("sensor_name");

    if (sensor_name.empty())
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(),
        "Parameter 'sensor_name' is empty. Please specify a sensor name.");
      return controller_interface::CallbackReturn::ERROR;
    }

    gain_roll_p = get_node()->declare_parameter<double>("gains.roll.p", 1);
    gain_roll_d = get_node()->declare_parameter<double>("gains.roll.d", 0);
    gain_pitch_p = get_node()->declare_parameter<double>("gains.pitch.p", 1);
    gain_pitch_d = get_node()->declare_parameter<double>("gains.pitch.d", 0);
    gain_yaw_p = get_node()->declare_parameter<double>("gains.yaw_rate.p", 1);

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
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
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

    // direction of body frame z-axis in world frame
    const Eigen::Vector3d z_state = q_state * Eigen::Vector3d::UnitZ();

    // direction of reference z-axis in world frame
    const double roll_ref = reference_interfaces_[0];
    const double pitch_ref = reference_interfaces_[1];
    const Eigen::Vector3d z_ref{
      std::sin(pitch_ref),
      -std::sin(roll_ref) * std::cos(pitch_ref),
      std::cos(roll_ref) * std::cos(pitch_ref),
    };

    // rotate the current z-axis to the reference z-axis
    const Eigen::Quaterniond q_err = Eigen::Quaterniond::FromTwoVectors(z_state, z_ref);

    const Eigen::Quaterniond q_err_A =
      (q_err.w() < 0) ? Eigen::Quaterniond{-q_err.coeffs()} : q_err;

    const double roll_err = q_err_A.vec()[0];
    const double pitch_err = q_err_A.vec()[1];

    assert(command_interfaces_.size() == 3);

    bool status_ok = true;

    // attitude control for roll and pitch
    status_ok &= command_interfaces_[0].set_value(gain_roll_p * roll_err + gain_roll_d * -wx);
    status_ok &= command_interfaces_[1].set_value(gain_pitch_p * pitch_err + gain_pitch_d * -wy);
    // yaw rate control
    status_ok &= command_interfaces_[2].set_value(gain_yaw_p * (reference_interfaces_[2] - wz));

    RCLCPP_DEBUG_STREAM(
      get_node()->get_logger(),
      std::fixed << std::setprecision(2) << std::endl
                 << "roll: " << reference_interfaces_[0] << " > " << roll_err << " -> "
                 << command_interfaces_[0].get_optional().value_or(nan) << " [rad/s]" << std::endl
                 << "pitch: " << reference_interfaces_[1] << " > " << pitch_err << " -> "
                 << command_interfaces_[1].get_optional().value_or(nan) << " [rad/s]" << std::endl
                 << "yaw_rate: " << reference_interfaces_[2] << " > " << wz << " -> "
                 << command_interfaces_[2].get_optional().value_or(nan) << " [rad/s]");

    return controller_interface::return_type(!status_ok);
  }

private:
  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();

  std::string mixer_name;
  std::string sensor_name;

  double gain_roll_p;
  double gain_roll_d;
  double gain_pitch_p;
  double gain_pitch_d;
  double gain_yaw_p;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_reference;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Vector3> msg_reference;
};

}  // namespace attitude

}  // namespace uav_controllers

PLUGINLIB_EXPORT_CLASS(
  uav_controllers::attitude::AttitudePID, controller_interface::ChainableControllerInterface)
