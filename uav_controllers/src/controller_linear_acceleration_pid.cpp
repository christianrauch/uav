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
    reference_interfaces_.resize(exported_reference_interface_names_.size(), nan);

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
    // reference: linear acceleration
    const Eigen::Vector3d acc_ref{
      reference_interfaces_[0],
      reference_interfaces_[1],
      reference_interfaces_[2],
    };

    // state: linear acceleration
    const Eigen::Vector3d acc_state{
      state_interfaces_[0].get_optional().value_or(nan),
      state_interfaces_[1].get_optional().value_or(nan),
      state_interfaces_[2].get_optional().value_or(nan),
    };

    // state: orientation
    const Eigen::Quaterniond q_state{
      state_interfaces_[6].get_optional().value_or(nan),  // w
      state_interfaces_[3].get_optional().value_or(nan),  // x
      state_interfaces_[4].get_optional().value_or(nan),  // y
      state_interfaces_[5].get_optional().value_or(nan),  // z
    };

    // DBG
    return controller_interface::return_type::OK;
  }

private:
  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();
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
