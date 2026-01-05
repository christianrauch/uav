#include <hardware_interface/actuator_interface.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace uav::proxy
{

class ProxyActuator : public hardware_interface::ActuatorInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    const CallbackReturn bcr = hardware_interface::ActuatorInterface::on_init(params);

    if (bcr != CallbackReturn::SUCCESS)
    {
      return bcr;
    }

    parse_command_interface_descriptions(params.hardware_info.joints, joint_command_interfaces_);

    // determine the number of motors from the number of command interfaces
    nrotors = std::count_if(
      joint_command_interfaces_.cbegin(), joint_command_interfaces_.cend(),
      [](const auto & iface) {
        return iface.second.get_prefix_name().substr(0, actuator_prefix.size()) == actuator_prefix;
      });

    pub_actuator =
      get_node()->create_publisher<mavros_msgs::msg::ActuatorControl>("~/actuators", 1);

    pub_rt_actuator =
      std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::ActuatorControl>>(
        pub_actuator);

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override
  {
    msg_ac.header.stamp = time;
    for (uint8_t i = 0; i < nrotors; i++)
    {
      msg_ac.controls[i] = get_command<double>(
        actuator_prefix + "/" + std::to_string(i + 1) + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    while (!pub_rt_actuator->try_publish(msg_ac));

    return hardware_interface::return_type::OK;
  }

private:
  static const std::string actuator_prefix;
  uint8_t nrotors = 0;
  rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr pub_actuator;
  realtime_tools::RealtimePublisher<mavros_msgs::msg::ActuatorControl>::UniquePtr pub_rt_actuator;
  mavros_msgs::msg::ActuatorControl msg_ac;
};

const std::string ProxyActuator::actuator_prefix = "rotor";

}  // namespace uav::proxy

PLUGINLIB_EXPORT_CLASS(uav::proxy::ProxyActuator, hardware_interface::ActuatorInterface)
