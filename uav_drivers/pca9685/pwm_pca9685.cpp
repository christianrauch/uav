#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <vector>

extern "C"
{
#include <i2c/smbus.h>
}

namespace uav::drivers
{
namespace pwm
{

std::string hex(const uint8_t value)
{
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(value);
  return oss.str();
}

class PCA9685 : public hardware_interface::ActuatorInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (const CallbackReturn bcr = hardware_interface::ActuatorInterface::on_init(params);
        bcr != CallbackReturn::SUCCESS)
    {
      return bcr;
    }

    if (params.hardware_info.rw_rate > max_pwm_rate)
    {
      RCLCPP_WARN_STREAM(
        get_logger(), "Actuator '"
                        << params.hardware_info.name << "' ("
                        << params.hardware_info.hardware_plugin_name
                        << ") is configured with a 'rw_rate' of " << params.hardware_info.rw_rate
                        << " Hz. The maximum PWM rate will be limited to " << max_pwm_rate
                        << " Hz. Reduce 'rw_rate' to silence this warning.");
    }

    const double pwm_rate = std::min<double>(params.hardware_info.rw_rate, max_pwm_rate);  // [Hz]
    step_per_us = steps * pwm_rate / 1'000'000;  // [LSB/μs]

    nrotors = std::count_if(
      joint_command_interfaces_.cbegin(), joint_command_interfaces_.cend(),
      [](const auto & iface) {
        return iface.second.get_prefix_name().substr(0, actuator_prefix.size()) == actuator_prefix;
      });

    if (nrotors > max_pwm_channls)
    {
      RCLCPP_FATAL_STREAM(
        get_logger(), "PCA9685 only supports up to '" << max_pwm_channls << "' PWM channels");
      return hardware_interface::CallbackReturn::ERROR;
    }

    rotor_cmds.resize(nrotors);

    if (!info_.hardware_parameters.contains("bus"))
    {
      RCLCPP_FATAL_STREAM(get_logger(), "PCA9685 parameter 'bus' not set");
      return hardware_interface::CallbackReturn::ERROR;
    }

    const int bus_id = std::stoi(info_.hardware_parameters["bus"]);

    // connect to I2C bus
    if ((fd = open(std::string("/dev/i2c-" + std::to_string(bus_id)).c_str(), O_RDWR)) < 0)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to open the i2c bus:" << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    if ((ioctl(fd, I2C_SLAVE, A_PCA9685)) < 0)
    {
      RCLCPP_FATAL_STREAM(
        get_logger(), std::hex << "Could not open PCA9685 at address " << hex(A_PCA9685) << ":"
                               << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    // reset MODE1 register to default values
    if (i2c_smbus_write_byte_data(fd, R_MODE1, 0x00) < 0)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to reset MODE1 register: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    // set AI bit: enable register Auto-Increment in order to write multiple registers at once
    if (i2c_smbus_write_byte_data(fd, R_MODE1, 0b0010'0000) < 0)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to set AI bit: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    // set OUTDRV bit: outputs are configured with a totem pole structure
    if (i2c_smbus_write_byte_data(fd, R_MODE2, 0b0000'0100) < 0)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to set OUTDRV bit: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    // prescale value for PWM frequency
    const uint8_t prescale_value = std::round(25'000'000 / (steps * pwm_rate) - 1);

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to
    // logic 1.

    const uint8_t oldmode = i2c_smbus_read_byte_data(fd, R_MODE1);
    if (errno != 0)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to read MODE1 register: " << strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }

    // clear RESTART bit and set SLEEP bit
    i2c_smbus_write_byte_data(fd, R_MODE1, (oldmode & 0b0111'1111) | 0b0001'0000);

    // set PRE_SCALE
    i2c_smbus_write_byte_data(fd, R_PRE_SCALE, prescale_value);

    // restart / wake up
    // The SLEEP bit must be logic 0 for at least 500 μs, before a logic 1 is written into the
    // RESTART bit.
    i2c_smbus_write_byte_data(fd, R_MODE1, oldmode);
    usleep(5000);
    // set RESTART bit to 1
    i2c_smbus_write_byte_data(fd, R_MODE1, oldmode | 0x80);

    RCLCPP_INFO_STREAM(
      get_logger(), "Initialised PCA9685 at I2C address " << hex(A_PCA9685) << " with "
                                                          << uint16_t(nrotors) << " PWM channels @ "
                                                          << pwm_rate << " Hz");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // reset all PWM values to 0
    constexpr std::array<uint8_t, 4> buf_zero = {0x00, 0x00, 0x00, 0x00};
    if (i2c_smbus_write_i2c_block_data(fd, R_ALL_LED_ON_L, buf_zero.size(), buf_zero.data()) < 0)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Failed to reset PWM values: " << strerror(errno));
    }

    count_buf = {0x00, 0x00, 0x00, 0x00};

    close(fd);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < nrotors; ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        actuator_prefix + "/" + std::to_string(i + 1), hardware_interface::HW_IF_VELOCITY,
        &rotor_cmds[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < nrotors; ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        actuator_prefix + "/" + std::to_string(i + 1), hardware_interface::HW_IF_VELOCITY,
        &rotor_cmds[i]));
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    for (size_t i = 0; i < nrotors; ++i)
    {
      const uint16_t step_off = duty_cycle_step_off(rotor_cmds[i]);

      count_buf[2] = (step_off >> 0) & 0xFF;  // OFF Low Byte (LEDx_OFF_L)
      count_buf[3] = (step_off >> 8) & 0x1F;  // OFF High Byte (LEDx_OFF_H)

      if (
        i2c_smbus_write_i2c_block_data(
          fd, R_LED0_ON_L + 4 * i, count_buf.size(), count_buf.data()) < 0)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to write PWM values: " << strerror(errno));
      }
    }

    return hardware_interface::return_type::OK;
  }

private:
  uint16_t duty_cycle_step_off(const double value)
  {
    // scale [0, 1] to [min_us, max_us]
    const double pulse_width_us = min_us + (max_us - min_us) * std::clamp<double>(value, 0, 1);

    // convert cycle start and stop times to start and stop steps
    // the cycle START is always at 0
    // the cycle STOP is at pulse_width_us
    return std::round(pulse_width_us * step_per_us);
  }

  static const std::string actuator_prefix;
  uint8_t nrotors = 0;
  std::vector<double> rotor_cmds;

  // buffer for PWM duty cycle ON/OFF counts [LEDx_ON_L, LEDx_ON_H, LEDx_OFF_L, LEDx_OFF_H]
  // ON count [LEDx_ON_L, LEDx_ON_H] is initialised to 0, PWM duty cycle always starts at 0
  std::array<uint8_t, 4> count_buf = {0x00, 0x00, 0x00, 0x00};

  // PWM cycle settings
  // At 2ms maximum pulse width, the theoretical PWM frequency is 500Hz. To ensure that the ESC
  // detects the falling edge of the PWM signal, we have to leave a gap.
  static constexpr double max_pwm_rate = 490;  // [Hz]
  static constexpr uint8_t max_pwm_channls = 16;
  static constexpr uint16_t steps = 1 << 12;  // 12-bit resolution
  double step_per_us = 0;                     // [LSB/μs]

  // pulse range [μs]
  static constexpr double min_us = 1000;
  static constexpr double max_us = 2000;

  int fd = 0;

  static constexpr uint8_t A_PCA9685 = 0x40;

  static constexpr uint8_t R_MODE1 = 0x00;
  static constexpr uint8_t R_MODE2 = 0x01;

  static constexpr uint8_t R_PRE_SCALE = 0xFE;

  // first LED output register
  static constexpr uint8_t R_LED0_ON_L = 0x06;

  static constexpr uint8_t R_ALL_LED_ON_L = 0xFA;
  static constexpr uint8_t R_ALL_LED_ON_H = 0xFB;
  static constexpr uint8_t R_ALL_LED_OFF_L = 0xFC;
  static constexpr uint8_t R_ALL_LED_OFF_H = 0xFD;
};

const std::string PCA9685::actuator_prefix = "rotor";

}  // namespace pwm
}  // namespace uav::drivers

PLUGINLIB_EXPORT_CLASS(uav::drivers::pwm::PCA9685, hardware_interface::ActuatorInterface)
