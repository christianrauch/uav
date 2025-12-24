#include <hardware_interface/sensor_interface.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace uav_hardware
{

class ProxyRC : public hardware_interface::SensorInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    const CallbackReturn bcr = hardware_interface::SensorInterface::on_init(params);

    if (bcr != CallbackReturn::SUCCESS)
    {
      return bcr;
    }

    sub_rc = get_node()->create_subscription<mavros_msgs::msg::OverrideRCIn>(
      "~/rc", 1,
      [this](const mavros_msgs::msg::OverrideRCIn::SharedPtr msg) -> void { msg_rc.set(*msg); });

    // check if sensor exist
    const bool has_sensor =
      params.hardware_info.sensors.cend() !=
      std::find_if(
        params.hardware_info.sensors.cbegin(), params.hardware_info.sensors.cend(),
        [this](const hardware_interface::ComponentInfo & info)
        { return info.name == sensor_name; });

    if (!has_sensor)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "missing sensor '" << sensor_name << "'");
      return CallbackReturn::ERROR;
    }

    nchannels = std::count_if(
      sensor_state_interfaces_.cbegin(), sensor_state_interfaces_.cend(),
      [](const auto & iface)
      {
        return iface.second.get_prefix_name() == sensor_name &&
               iface.second.get_interface_name().substr(0, state_prefix.size()) == state_prefix;
      });

    if (nchannels > max_channels)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "can only provide up to " << max_channels << " cannels");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (const std::optional<mavros_msgs::msg::OverrideRCIn> msg = msg_rc.try_get())
    {
      for (uint8_t i = 0; i < nchannels; i++)
      {
        set_state<double>(
          sensor_name + "/" + state_prefix + "/" + std::to_string(i + 1), msg->channels[i]);
      }
    }

    return hardware_interface::return_type::OK;
  }

private:
  static const std::string sensor_name;
  static const std::string state_prefix;
  static constexpr std::size_t max_channels =
    std::tuple_size_v<mavros_msgs::msg::OverrideRCIn::_channels_type>;
  uint8_t nchannels = 0;
  rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr sub_rc;
  realtime_tools::RealtimeThreadSafeBox<mavros_msgs::msg::OverrideRCIn> msg_rc;
};

const std::string ProxyRC::sensor_name = "rc";
const std::string ProxyRC::state_prefix = "channel";

}  // namespace uav_hardware

PLUGINLIB_EXPORT_CLASS(uav_hardware::ProxyRC, hardware_interface::SensorInterface)
