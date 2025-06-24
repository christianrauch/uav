#include <hardware_interface/sensor_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace uav_hardware
{

class ProxyIMU : public hardware_interface::SensorInterface
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

    sub_imu = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      "~/imu", 1, [](sensor_msgs::msg::Imu::SharedPtr) {});

    const std::string sensor_name_param = "sensor_name";

    try
    {
      sensor_name = params.hardware_info.hardware_parameters.at(sensor_name_param);
    }
    catch (std::out_of_range &)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "parameter '" << sensor_name_param << "' not set");
      return CallbackReturn::ERROR;
    }

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

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (sub_imu->take(msg_imu, msg_imu_info))
    {
      set_state<double>(sensor_name + "/orientation.x", msg_imu.orientation.x);
      set_state<double>(sensor_name + "/orientation.y", msg_imu.orientation.y);
      set_state<double>(sensor_name + "/orientation.z", msg_imu.orientation.z);
      set_state<double>(sensor_name + "/orientation.w", msg_imu.orientation.w);

      set_state<double>(sensor_name + "/angular_velocity.x", msg_imu.angular_velocity.x);
      set_state<double>(sensor_name + "/angular_velocity.y", msg_imu.angular_velocity.y);
      set_state<double>(sensor_name + "/angular_velocity.z", msg_imu.angular_velocity.z);

      set_state<double>(sensor_name + "/linear_acceleration.x", msg_imu.linear_acceleration.x);
      set_state<double>(sensor_name + "/linear_acceleration.y", msg_imu.linear_acceleration.y);
      set_state<double>(sensor_name + "/linear_acceleration.z", msg_imu.linear_acceleration.z);

      return hardware_interface::return_type::OK;
    }

    return hardware_interface::return_type::OK;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  std::string sensor_name;
  sensor_msgs::msg::Imu msg_imu;
  rclcpp::MessageInfo msg_imu_info;
};

}  // namespace uav_hardware

PLUGINLIB_EXPORT_CLASS(uav_hardware::ProxyIMU, hardware_interface::SensorInterface)
