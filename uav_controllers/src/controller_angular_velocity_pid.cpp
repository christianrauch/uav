#include <control_toolbox/pid.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace uav_controllers
{

namespace angular_velocity
{

class AngVelPID : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    const std::vector<std::string> interface_names = {
      mixer_name + "/" + cmd_order[0],
      mixer_name + "/" + cmd_order[1],
      mixer_name + "/" + cmd_order[2],
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
    // set reference interface
    exported_reference_interface_names_ = {
      "angular_velocity.x",
      "angular_velocity.y",
      "angular_velocity.z",
    };
    reference_interfaces_.resize(exported_reference_interface_names_.size(), nan);

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
        msg->x,  // roll
        msg->y,  // pitch
        msg->z,  // yaw
      };
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    // roll, pitch, yaw
    std::array<double, cmd_order.size()> cmds;
    for (size_t i = 0; i < pid_controllers.size(); i++)
    {
      const double state = state_interfaces_[i].get_optional().value_or(nan);
      cmds[i] = pid_controllers[i]->compute_command(reference_interfaces_[i] - state, period);
      RCLCPP_DEBUG_STREAM(
        get_node()->get_logger(), std::fixed << std::setprecision(2) << cmd_order[i] << ": "
                                             << reference_interfaces_[i] << " <> " << state
                                             << " -> " << cmds[i] << " [rad/s]");
    }

    bool status_ok = true;

    for (size_t i = 0; i < cmds.size(); i++)
    {
      status_ok &= command_interfaces_[i].set_value(cmds[i]);
    }

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
  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();

  // the command interface order relates to the corresponding order of the state
  // roll  - angular_velocity.x
  // pitch - angular_velocity.y
  // yaw   - angular_velocity.z
  static constexpr std::array<std::string, 3> cmd_order = {"roll", "pitch", "yaw"};

  std::string mixer_name;
  std::string sensor_name;
  std::array<std::shared_ptr<control_toolbox::Pid>, cmd_order.size()> pid_controllers;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_reference;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Vector3> msg_reference;
};

}  // namespace angular_velocity

}  // namespace uav_controllers

PLUGINLIB_EXPORT_CLASS(
  uav_controllers::angular_velocity::AngVelPID, controller_interface::ChainableControllerInterface)
