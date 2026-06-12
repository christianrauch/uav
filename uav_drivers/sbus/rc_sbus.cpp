#include <asm/termbits.h>
#include <fcntl.h>
#include <linux/serial.h>
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

    if (params.hardware_info.rw_rate > max_sbus_rate)
    {
      RCLCPP_FATAL_STREAM(
        get_logger(), std::format(
                        "S.BUS interface '{}' ({}) is configured with a 'rw_rate' of {} Hz. "
                        "The maximum S.BUS framerate is {} Hz. Reduce 'rw_rate'.",
                        params.hardware_info.name, params.hardware_info.hardware_plugin_name,
                        params.hardware_info.rw_rate, max_sbus_rate));
      return hardware_interface::CallbackReturn::ERROR;
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
      return hardware_interface::CallbackReturn::ERROR;
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
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!info_.hardware_parameters.contains("device"))
    {
      RCLCPP_FATAL_STREAM(get_logger(), "SBUS 'device' not set");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if ((fd = open(info_.hardware_parameters["device"].c_str(), O_RDONLY | O_NOCTTY)) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to open serial port: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (ioctl(fd, TCSETS2, &options) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to configure serial port: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    struct serial_struct ser_info;
    if (ioctl(fd, TIOCGSERIAL, &ser_info) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "TIOCGSERIAL failed: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    ser_info.flags |= ASYNC_LOW_LATENCY;

    if (ioctl(fd, TIOCSSERIAL, &ser_info) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "TIOCSSERIAL failed: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
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
    if (!sbus_read_frame())
    {
      return hardware_interface::return_type::OK;
    }

    sbus_channels_decode();

    // boolean channels
    channels_digital[0] = frame[23] & 0x01;
    channels_digital[1] = frame[23] & 0x02;

    // lost frame and failsafe (multiple lost frames) indicators
    const bool frame_lost = frame[23] & 0x04;
    const bool failsafe = frame[23] & 0x08;

    set_state<bool>(std::string{sensor_name} + "/failsafe", failsafe);

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
      set_state<double>(
        std::string{sensor_name} + "/" + std::string{state_prefix} + "/" + std::to_string(i + 1),
        val);
    }

    return hardware_interface::return_type::OK;
  }

private:
  bool sbus_read_frame()
  {
    const int n = ::read(fd, frame.data(), frame.size());

    if (n == -1)
    {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error reading from serial port: " << strerror(errno) << std::endl);
      return false;
    }

    if (n == 0)
    {
      return false;
    }

    // we expect to read exactly one S.BUS frame, skip otherwise
    if (n != N || frame[0] != SBUS_HEADER || frame[24] != SBUS_END)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Invalid S.BUS frame!");
      return false;
    }

    return true;
  }

  void sbus_channels_decode()
  {
    channels[0] = (uint16_t)((frame[1] | frame[2] << 8) & 0x07FF);
    channels[1] = (uint16_t)((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
    channels[2] = (uint16_t)((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
    channels[3] = (uint16_t)((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
    channels[4] = (uint16_t)((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
    channels[5] = (uint16_t)((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF);
    channels[6] = (uint16_t)((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
    channels[7] = (uint16_t)((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
    channels[8] = (uint16_t)((frame[12] | frame[13] << 8) & 0x07FF);
    channels[9] = (uint16_t)((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
    channels[10] = (uint16_t)((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
    channels[11] = (uint16_t)((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
    channels[12] = (uint16_t)((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
    channels[13] = (uint16_t)((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
    channels[14] = (uint16_t)((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
    channels[15] = (uint16_t)((frame[21] >> 5 | frame[22] << 3) & 0x07FF);
  }

  static constexpr std::string_view sensor_name = "rc";
  static constexpr std::string_view state_prefix = "channel";
  static constexpr std::size_t max_channels = 16;
  uint8_t nchannels = 0;

  // SBUS frame
  static constexpr size_t N = 25;
  std::array<uint8_t, N> frame;
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

  // 8E2: 8/12 bits used for SBUS frame
  static constexpr double max_sbus_rate = options.c_ospeed * (8. / 12.) / (N * 8);
};

}  // namespace rc
}  // namespace uav::drivers

PLUGINLIB_EXPORT_CLASS(uav::drivers::rc::SBUS, hardware_interface::SensorInterface)
