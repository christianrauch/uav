#include <asm/termbits.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <hardware_interface/sensor_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace uav::drivers
{
namespace rc
{

class SBUS : public hardware_interface::SensorInterface
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

    if (!info_.hardware_parameters.contains("device"))
    {
      RCLCPP_FATAL_STREAM(get_logger(), "SBUS 'device' not set");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if ((fd = open(info_.hardware_parameters["device"].c_str(), O_RDONLY | O_NOCTTY)) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to open serial port: " << strerror(errno));
    }

    if (ioctl(fd, TCSETS2, &options) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to configure serial port: " << strerror(errno));
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    close(fd);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    const int n = ::read(fd, &buffer, sizeof(buffer));

    if (n == 0)
    {
      return hardware_interface::return_type::OK;
    }

    // we expect to read exactly one S.BUS frame, skip otherwise
    if (n != N || buffer[0] != SBUS_HEADER || buffer[24] != SBUS_END)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Invalid S.BUS frame!");
      return hardware_interface::return_type::OK;
    }

    sbus_channels_decode(buffer + 1);

    // boolean channels
    channels_digital[0] = buffer[23] & 0x01;
    channels_digital[1] = buffer[23] & 0x02;

    // lost frame and failsafe (multiple lost frames) indicators
    const bool frame_lost = buffer[23] & 0x04;
    const bool failsafe = buffer[23] & 0x08;

    // show connection lost/recovered messages once
    if (!connection_lost && failsafe)
    {
      connection_lost = true;
      RCLCPP_ERROR_STREAM(get_logger(), "Connection lost!");
    }
    else if (connection_lost && !failsafe)
    {
      connection_lost = false;
      RCLCPP_ERROR_STREAM(get_logger(), "Connection recovered!");
    }

    if (frame_lost || failsafe)
    {
      return hardware_interface::return_type::OK;
    }

    for (uint8_t i = 0; i < nchannels; i++)
    {
      const double val = static_cast<double>(channels[i] - umin) / (umax - umin);
      set_state<double>(sensor_name + "/" + state_prefix + "/" + std::to_string(i + 1), val);
    }

    return hardware_interface::return_type::OK;
  }

private:
  void sbus_channels_decode(const uint8_t * payload)
  {
    channels[0] = (uint16_t)((payload[0] | payload[1] << 8) & 0x07FF);
    channels[1] = (uint16_t)((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
    channels[2] = (uint16_t)((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
    channels[3] = (uint16_t)((payload[4] >> 1 | payload[5] << 7) & 0x07FF);
    channels[4] = (uint16_t)((payload[5] >> 4 | payload[6] << 4) & 0x07FF);
    channels[5] = (uint16_t)((payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF);
    channels[6] = (uint16_t)((payload[8] >> 2 | payload[9] << 6) & 0x07FF);
    channels[7] = (uint16_t)((payload[9] >> 5 | payload[10] << 3) & 0x07FF);
    channels[8] = (uint16_t)((payload[11] | payload[12] << 8) & 0x07FF);
    channels[9] = (uint16_t)((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
    channels[10] = (uint16_t)((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
    channels[11] = (uint16_t)((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
    channels[12] = (uint16_t)((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
    channels[13] = (uint16_t)((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
    channels[14] = (uint16_t)((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
    channels[15] = (uint16_t)((payload[20] >> 5 | payload[21] << 3) & 0x07FF);
  }

  static const std::string sensor_name;
  static const std::string state_prefix;
  static constexpr std::size_t max_channels = 16;
  uint8_t nchannels = 0;

  // SBUS frame
  static constexpr size_t N = 25;
  uint8_t buffer[N];
  static constexpr uint8_t SBUS_HEADER = 0x0F;
  static constexpr uint8_t SBUS_END = 0x0;
  std::array<uint16_t, max_channels> channels;  // 1 - 16: 11 bit
  std::array<bool, 2> channels_digital;         // 17, 18: 1 bit

  // 11 bit : 2^11 = 2048
  static constexpr uint16_t umin = 172;
  static constexpr uint16_t umax = 1811;

  bool connection_lost = false;

  int fd = 0;

  static constexpr struct termios2 options = []
  {
    // https://github.com/bolderflight/sbus/blob/main/README.md
    // - baud rate of 100000
    // - 8 data bits
    // - even parity
    // - 2 stop bits
    struct termios2 opt;
    opt.c_cflag = CS8 | CSTOPB | PARENB | CLOCAL | CREAD | BOTHER;
    opt.c_ospeed = 100000;
    return opt;
  }();
};

const std::string SBUS::sensor_name = "rc";
const std::string SBUS::state_prefix = "channel";

}  // namespace rc
}  // namespace uav::drivers

PLUGINLIB_EXPORT_CLASS(uav::drivers::rc::SBUS, hardware_interface::SensorInterface)
