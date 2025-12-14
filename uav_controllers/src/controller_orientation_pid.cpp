#include <Eigen/Geometry>
#include <control_toolbox/pid.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace uav_controllers
{

namespace angular_position
{

class AngPosPID : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      velocity_controller_name + "/angular_velocity.x",
      velocity_controller_name + "/angular_velocity.y",
      velocity_controller_name + "/angular_velocity.z",
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      sensor_name + "/orientation.x",
      sensor_name + "/orientation.y",
      sensor_name + "/orientation.z",
      sensor_name + "/orientation.w",
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    // set reference interface
    exported_reference_interface_names_ = {
      "orientation.x",
      "orientation.y",
      "orientation.z",
      "orientation.w",
    };
    reference_interfaces_.resize(exported_reference_interface_names_.size(), nan);

    sub_reference = get_node()->create_subscription<geometry_msgs::msg::Quaternion>(
      "~/reference", 1, [this](const geometry_msgs::msg::Quaternion::SharedPtr msg) -> void
      { msg_reference.set(*msg); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    velocity_controller_name =
      get_node()->declare_parameter<std::string>("velocity_controller_name");

    if (velocity_controller_name.empty())
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(),
        "Parameter 'velocity_controller_name' is empty. Please specify a velocity controller.");
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

    tau = get_node()->declare_parameter<double>("gains.tau", 2);

    control_toolbox::AntiWindupStrategy antiwindup;
    antiwindup.type = control_toolbox::AntiWindupStrategy::NONE;

    for (size_t i = 0; i < cmd_order.size(); i++)
    {
      // gains
      const double gp = get_node()->declare_parameter<double>("gains." + cmd_order[i] + ".p", 1);
      const double gi = get_node()->declare_parameter<double>("gains." + cmd_order[i] + ".i", 0);
      const double gd = get_node()->declare_parameter<double>("gains." + cmd_order[i] + ".d", 0);

      pid_controllers[i] = std::make_shared<control_toolbox::Pid>(
        gp, gi, gd, std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), antiwindup);
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
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
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

    bool status_ok = true;

    for (Eigen::Index i = 0; i < o.size(); i++)
    {
      const double cmd = pid_controllers[i]->compute_command(o[i], period);
      RCLCPP_DEBUG_STREAM(
        get_node()->get_logger(), std::fixed << std::setprecision(2) << cmd_order[i] << ": " << o[i]
                                             << " -> " << cmd << " [rad]");
      status_ok &= command_interfaces_[i].set_value(cmd);
    }

    if (status_ok)
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
  }

private:
  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();

  static constexpr std::array<std::string, 3> cmd_order = {"roll", "pitch", "yaw"};

  std::string velocity_controller_name;
  std::string sensor_name;
  double tau;
  std::array<std::shared_ptr<control_toolbox::Pid>, cmd_order.size()> pid_controllers;

  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_reference;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Quaternion> msg_reference;
};

}  // namespace angular_position

}  // namespace uav_controllers

PLUGINLIB_EXPORT_CLASS(
  uav_controllers::angular_position::AngPosPID, controller_interface::ChainableControllerInterface)
