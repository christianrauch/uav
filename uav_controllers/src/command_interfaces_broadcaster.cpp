#include <control_msgs/msg/float64_values.hpp>
#include <control_msgs/msg/keys.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <format>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <string>
#include <uav_controllers/command_interfaces_broadcaster_parameters.hpp>

namespace command_interfaces_broadcaster
{

class CommandInterfacesBroadcaster : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL, command_interface_names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {};
  }

  controller_interface::CallbackReturn on_init() override
  {
    param_listener = std::make_shared<ParamListener>(get_node());

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const Params params = param_listener->get_params();

    pub_names = get_node()->create_publisher<control_msgs::msg::Keys>(
      "~/names", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    pub_values = get_node()->create_publisher<control_msgs::msg::Float64Values>(
      "~/values", rclcpp::SystemDefaultsQoS());
    pub_rt_values =
      std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::Float64Values>>(
        pub_values);

    // setup command and reference interfaces
    command_interface_names.clear();
    for (const std::string & name : params.command_interfaces)
    {
      command_interface_names.push_back(std::format("{}/{}", params.controller_name, name));
    }

    exported_reference_interface_names_ = params.command_interfaces;
    reference_interfaces_.assign(exported_reference_interface_names_.size(), 0);

    // publish keys once (latched)
    control_msgs::msg::Keys msg_names;
    msg_names.header.stamp = get_node()->now();
    msg_names.keys = exported_reference_interface_names_;
    pub_names->publish(msg_names);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override
  {
    // broadcast values
    if (pub_rt_values)
    {
      msg_values.header.stamp = time;
      msg_values.values = reference_interfaces_;
      pub_rt_values->try_publish(msg_values);
    }

    // forward values to command interfaces
    bool status_ok = true;
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
    {
      status_ok &= command_interfaces_[i].set_value(reference_interfaces_[i]);
    }

    return status_ok ? controller_interface::return_type::OK
                     : controller_interface::return_type::ERROR;
  }

protected:
  std::shared_ptr<ParamListener> param_listener;

  std::vector<std::string> command_interface_names;

  rclcpp::Publisher<control_msgs::msg::Keys>::SharedPtr pub_names;
  rclcpp::Publisher<control_msgs::msg::Float64Values>::SharedPtr pub_values;
  realtime_tools::RealtimePublisher<control_msgs::msg::Float64Values>::UniquePtr pub_rt_values;
  control_msgs::msg::Float64Values msg_values;
};

}  // namespace command_interfaces_broadcaster

PLUGINLIB_EXPORT_CLASS(
  command_interfaces_broadcaster::CommandInterfacesBroadcaster,
  controller_interface::ChainableControllerInterface)
