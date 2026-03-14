#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace uav::controllers::arming
{

class ControllerArming : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> interface_names = {
      "rc/channel/" + std::to_string(arming_channel_)
    };
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    exported_state_interface_names_ = {"active"};
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    arming_channel_ = get_node()->declare_parameter<int64_t>("arming_channel", 5);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool on_set_chained_mode(bool /*chained_mode*/) override { return true; }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // read RC channel
    double channel_value = state_interfaces_.front().get_optional().value_or(0.0);
    
    // set exported state interface value to 1 if channel > 0.5, else 0
    state_interfaces_values_[0] = channel_value > 0.5 ? 1.0 : 0.0;
    
    return controller_interface::return_type::OK;
  }

private:
  int64_t arming_channel_ = 5;
};

}  // namespace uav::controllers::arming

PLUGINLIB_EXPORT_CLASS(
  uav::controllers::arming::ControllerArming, controller_interface::ChainableControllerInterface)
