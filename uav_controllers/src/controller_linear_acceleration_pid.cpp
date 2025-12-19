#include <Eigen/Geometry>
#include <control_toolbox/pid.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace uav_controllers
{

namespace linear_acceleration
{

class LinAccPID : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      orientation_controller_name + "/orientation.x",
      orientation_controller_name + "/orientation.y",
      orientation_controller_name + "/orientation.z",
      orientation_controller_name + "/orientation.w",
      mixer_name + "/thrust",
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      // linear acceleration
      sensor_name + "/linear_acceleration.x",
      sensor_name + "/linear_acceleration.y",
      sensor_name + "/linear_acceleration.z",
      // orientation
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
      "linear_acceleration.x",
      "linear_acceleration.y",
      "linear_acceleration.z",
    };
    // by default, do not accelerate in either direction
    reference_interfaces_.resize(exported_reference_interface_names_.size(), 0);

    sub_reference = get_node()->create_subscription<geometry_msgs::msg::Vector3>(
      "~/reference", 1, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) -> void
      { msg_reference.set(*msg); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    orientation_controller_name =
      get_node()->declare_parameter<std::string>("orientation_controller_name");

    if (orientation_controller_name.empty())
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(),
        "Parameter 'velocity_controller_name' is empty. Please specify a velocity controller.");
      return controller_interface::CallbackReturn::ERROR;
    }

    mixer_name = get_node()->declare_parameter<std::string>("mixer");

    sensor_name = get_node()->declare_parameter<std::string>("sensor_name");

    if (sensor_name.empty())
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(),
        "Parameter 'sensor_name' is empty. Please specify a sensor name.");
      return controller_interface::CallbackReturn::ERROR;
    }

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
    if (const std::optional<geometry_msgs::msg::Vector3> msg = msg_reference.try_get())
    {
      reference_interfaces_ = {
        msg->x,
        msg->y,
        msg->z,
      };
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    // reference: linear acceleration (world frame)
    const Eigen::Vector3d acc_ref{
      reference_interfaces_[0],
      reference_interfaces_[1],
      reference_interfaces_[2],
    };

    // state: linear acceleration (body frame)
    const Eigen::Vector3d acc_state{
      state_interfaces_[0].get_optional().value_or(nan),
      state_interfaces_[1].get_optional().value_or(nan),
      state_interfaces_[2].get_optional().value_or(nan),
    };

    // state: orientation (world to body frame)
    const Eigen::Quaterniond q_state{
      state_interfaces_[6].get_optional().value_or(nan),  // w
      state_interfaces_[3].get_optional().value_or(nan),  // x
      state_interfaces_[4].get_optional().value_or(nan),  // y
      state_interfaces_[5].get_optional().value_or(nan),  // z
    };

    // rotate reference and state linear acceleration to the body frame
    const Eigen::Vector3d acc_ref_body = q_state.conjugate() * acc_ref;

    // linear acceleration error in body frame
    const Eigen::Vector3d acc_err_body = acc_ref_body - acc_state;

    // compute commands for x, y, z linear acceleration
    // std::array<double, cmd_order.size()> cmds;
    Eigen::Vector3d cmds;
    for (size_t i = 0; i < 3; ++i)
    {
      cmds[i] = pid_controllers[i]->compute_command(acc_err_body[i], period);
    }

    // TODO: setting the target orientation should work without PIDs

    // RCLCPP_INFO_STREAM(
    //   get_node()->get_logger(),
    //   std::fixed << std::setprecision(2) << "cmds (x,y,z): " << cmds.transpose());

    // The thrust command is the magnitude of the commanded linear acceleration in the body frame.
    // This is the output of the PID for the linear acceleration error.
    // The thrust is applied along the body's z-axis.
    const double thrust_z = pid_controllers[3]->compute_command(cmds.norm(), period);

    // The desired orientation is the rotation of the z-axis of the body frame
    // to align with the commanded linear acceleration vector.
    // The commanded linear acceleration is `cmds` in the body frame.
    // We want the body's z-axis to point in the direction of `cmds`.
    // The current body z-axis in the world frame is q_state * [0,0,1].
    // The desired body z-axis in the world frame is the normalized `cmds` vector.
    // However, `cmds` here are the output of the PID for linear acceleration error in the body
    // frame. This means `cmds` are the desired linear accelerations in the body frame. The
    // orientation controller expects a quaternion reference. The thrust command is also derived
    // from the linear acceleration.

    // Compute the quaternion that rotates the z-axis of the body frame to the `cmds` vector.
    const Eigen::Quaterniond q_cmd =
      Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitZ(), cmds.normalized());

    // RCLCPP_INFO_STREAM(
    //   get_node()->get_logger(), std::fixed << std::setprecision(2)
    //                                        << "q_cmd (x,y,z,w): " << q_cmd.coeffs().transpose()
    //                                        << " thrust_z: " << thrust_z);

    bool status_ok = true;

    // Set orientation commands
    status_ok &= command_interfaces_[0].set_value(q_cmd.x());
    status_ok &= command_interfaces_[1].set_value(q_cmd.y());
    status_ok &= command_interfaces_[2].set_value(q_cmd.z());
    status_ok &= command_interfaces_[3].set_value(q_cmd.w());

    // Set thrust command
    status_ok &= command_interfaces_[4].set_value(thrust_z);

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
  // The first three commands are for orientation.x, orientation.y, orientation.z
  static constexpr std::array<std::string, 4> cmd_order = {"x", "y", "z", "thrust"};

  std::string mixer_name;
  std::string orientation_controller_name;
  std::string sensor_name;
  std::array<std::shared_ptr<control_toolbox::Pid>, cmd_order.size()> pid_controllers;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_reference;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Vector3> msg_reference;
};

}  // namespace linear_acceleration

}  // namespace uav_controllers

PLUGINLIB_EXPORT_CLASS(
  uav_controllers::linear_acceleration::LinAccPID,
  controller_interface::ChainableControllerInterface)
