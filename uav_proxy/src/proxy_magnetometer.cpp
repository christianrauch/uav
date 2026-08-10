#include <hardware_interface/sensor_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace uav::proxy
{

class ProxyMagnetometer : public hardware_interface::SensorInterface
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

    sub_mag = get_node()->create_subscription<sensor_msgs::msg::MagneticField>(
      "~/magnetic_field", 1, [](sensor_msgs::msg::MagneticField::SharedPtr) {});

    static constexpr std::string_view sensor_name_param = "sensor_name";

    try
    {
      sensor_name = params.hardware_info.hardware_parameters.at(std::string{sensor_name_param});
    }
    catch (std::out_of_range &)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "parameter '" << sensor_name_param << "' not set");
      return CallbackReturn::ERROR;
    }

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

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (sub_mag->take(msg_mag, msg_mag_info))
    {
      set_state<double>(sensor_name + "/magnetic_field.x", msg_mag.magnetic_field.x);
      set_state<double>(sensor_name + "/magnetic_field.y", msg_mag.magnetic_field.y);
      set_state<double>(sensor_name + "/magnetic_field.z", msg_mag.magnetic_field.z);
    }

    return hardware_interface::return_type::OK;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag;
  std::string sensor_name;
  sensor_msgs::msg::MagneticField msg_mag;
  rclcpp::MessageInfo msg_mag_info;
};

}  // namespace uav::proxy

PLUGINLIB_EXPORT_CLASS(uav::proxy::ProxyMagnetometer, hardware_interface::SensorInterface)
