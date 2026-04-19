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

    if (nrotors > nrotors_max)
    {
      RCLCPP_FATAL_STREAM(
        get_logger(), "ProxyActuator only supports up to '" << nrotors_max << "' rotors");
      return hardware_interface::CallbackReturn::ERROR;
    }

    pub_actuator =
      get_node()->create_publisher<mavros_msgs::msg::ActuatorControl>("~/actuators", 1);

    pub_rt_actuator =
      std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::ActuatorControl>>(
        pub_actuator);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < nrotors; ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        std::string{actuator_prefix} + "/" + std::to_string(i + 1),
        hardware_interface::HW_IF_POSITION, &rotor_position[i]));
    }

    for (size_t i = 0; i < nrotors; ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        std::string{actuator_prefix} + "/" + std::to_string(i + 1),
        hardware_interface::HW_IF_VELOCITY, &rotor_velocity[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < nrotors; ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        std::string{actuator_prefix} + "/" + std::to_string(i + 1),
        hardware_interface::HW_IF_VELOCITY, &rotor_velocity[i]));
    }
    return command_interfaces;
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
      msg_ac.controls[i] = rotor_velocity[i];
    }

    while (!pub_rt_actuator->try_publish(msg_ac));

    return hardware_interface::return_type::OK;
  }

private:
  static constexpr std::string_view actuator_prefix = "rotor";
  static constexpr std::size_t nrotors_max =
    std::tuple_size<mavros_msgs::msg::ActuatorControl::_controls_type>::value;
  uint8_t nrotors = 0;
  rclcpp::Publisher<mavros_msgs::msg::ActuatorControl>::SharedPtr pub_actuator;
  realtime_tools::RealtimePublisher<mavros_msgs::msg::ActuatorControl>::UniquePtr pub_rt_actuator;
  mavros_msgs::msg::ActuatorControl msg_ac;
  std::array<double, nrotors_max> rotor_position;
  std::array<double, nrotors_max> rotor_velocity;
};

}  // namespace uav::proxy

PLUGINLIB_EXPORT_CLASS(uav::proxy::ProxyActuator, hardware_interface::ActuatorInterface)
