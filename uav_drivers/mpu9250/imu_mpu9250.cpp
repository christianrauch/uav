#include <pluginlib/class_list_macros.hpp>
#include <uav_drivers/imu_base.hpp>

extern "C"
{
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

namespace uav::drivers
{

namespace imu
{

std::string hex(const uint8_t value)
{
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(value);
  return oss.str();
}

class MPU9250 : public IMUInterfaceBase
{
private:
  uint8_t AK8963_read_reg(uint8_t reg)
  {
    // set I2C_SLV0_RNW to 1 (read); set I2C_ID_0 to address of AK8963
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_ADDR, 0x80 | A_AK8963);

    // set I2C_SLV0_REG to 'reg' register of AK8963
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_REG, reg);

    // set I2C_SLV0_EN; read 1 register
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_CTRL, 0b10000000 | 1);

    usleep(1000);

    return i2c_smbus_read_byte_data(fd, R_EXT_SENS_DATA_00);
  }

  void AK8963_write_reg(uint8_t reg, uint8_t value)
  {
    // set I2C_SLV0_RNW to 0 (write); set I2C_ID_0 to address of AK8963
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_ADDR, 0x00 | A_AK8963);

    // set I2C_SLV0_REG to 'reg' register of AK8963
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_REG, reg);

    // set value to data out (DO) register
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_DO, value);

    // set I2C_SLV0_EN; write 1 register
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_CTRL, 0b10000000 | 1);

    usleep(1000);
  }

  bool init_driver() override final
  {
    if (!info_.hardware_parameters.contains("bus"))
    {
      throw std::runtime_error("MPU9250 bus parameter not set");
    }

    const int bus_id = std::stoi(info_.hardware_parameters["bus"]);

    // connect to I2C bus
    if ((fd = open(std::string("/dev/i2c-" + std::to_string(bus_id)).c_str(), O_RDWR)) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to open the i2c bus:" << strerror(errno));
      return false;
    }

    if ((ioctl(fd, I2C_SLAVE, A_MPU6050)) < 0)
    {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        std::hex << "Could not open MPU at address " << hex(A_MPU6050) << ":" << strerror(errno));
      return false;
    }

    // check for MPU presence
    if (i2c_smbus_read_byte_data(fd, R_WHO_AM_I) != V_IAM_MPU)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not find MPU9250");
      return false;
    }

    RCLCPP_INFO_STREAM(
      get_logger(),
      std::hex << "Found MPU9250 (" << hex(V_IAM_MPU) << ") at address " << hex(A_MPU6050));

    // reset, set H_RESET bit
    i2c_smbus_write_byte_data(fd, R_PWR_MGMT_1, 0b10000000);
    usleep(1000);

    // set sample rate divider to 0 (1kHz)
    if (i2c_smbus_write_byte_data(fd, R_SMPLRT_DIV, 0) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to set sample rate divider: " << strerror(errno));
      return false;
    }

    // gyroscope low-pass filter setting
    constexpr uint8_t DLPF_CFG = 2;  // cutoff above 92 Hz
    if (i2c_smbus_write_byte_data(fd, R_CONFIG, DLPF_CFG) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to set DLPF: " << strerror(errno));
      return false;
    }

    // set gyroscope scale
    if (i2c_smbus_write_byte_data(fd, R_GYRO_CONFIG, GYRO_FS_SEL << 3) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to set gyroscope scale: " << strerror(errno));
      return false;
    }

    // set accelerometer scale
    if (i2c_smbus_write_byte_data(fd, R_ACCEL_CONFIG, ACCEL_FS_SEL << 3) < 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to set accelerometer scale: " << strerror(errno));
      return false;
    }

    // use MPU9250 as I2C master to read AK8963 magnetometer directly
    i2c_smbus_write_byte_data(fd, R_USER_CTRL, 0b00100000);  // set I2C_MST_EN to 1
    i2c_smbus_write_byte_data(fd, R_I2C_MST_CTRL, 13);  // set I2C_MST_CLK to 13: 8MHz/20 = 400kHz

    usleep(1000);

    // check for AK8963 presence
    const uint8_t AK8963_id = AK8963_read_reg(R_WIA);
    if (AK8963_id != V_IAM_AK8963)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not find AK8963");
      return false;
    }

    RCLCPP_INFO_STREAM(
      get_logger(), "Found AK8963 (" << hex(V_IAM_AK8963) << ") at address " << hex(A_AK8963));

    // soft reset AK8963 via SRST bit in CNTL2 register
    AK8963_write_reg(R_CNTL2, 0x01);

    // configure magnetometer
    // set to continuous measurement mode 2 (MODE: 0b0110, 100 Hz) with 16-bit output (BIT: 1)
    constexpr uint8_t MODE = 0b0110;
    constexpr uint8_t BIT = 0b1;
    AK8963_write_reg(R_CNTL1, (BIT << 4) | MODE);

    // read sensitivity adjustment values
    const uint8_t asa_x = AK8963_read_reg(R_ASAX);
    const uint8_t asa_y = AK8963_read_reg(R_ASAY);
    const uint8_t asa_z = AK8963_read_reg(R_ASAZ);

    // calculate magnetometer sensitivity
    constexpr double range = 2 * 4912;  // ± 4912 µT
    constexpr uint8_t bits = BIT ? 16 : 14;
    constexpr double sensitivity_scale_factor = range / (1 << bits);  // [µT/LSB]

    mag_sensitivity[0] = ((((asa_x - 128) * 0.5) / 128) + 1) * sensitivity_scale_factor;
    mag_sensitivity[1] = ((((asa_y - 128) * 0.5) / 128) + 1) * sensitivity_scale_factor;
    mag_sensitivity[2] = ((((asa_z - 128) * 0.5) / 128) + 1) * sensitivity_scale_factor;

    // set the MPU I2C master to continuously read magnetometer registers
    // set I2C_SLV0_RNW to 1 (read); set I2C_ID_0 to address of AK8963
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_ADDR, 0x80 | A_AK8963);
    // set I2C_SLV0_REG to HXL register of AK8963
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_REG, R_HXL);
    // set I2C_SLV0_EN; read 7 registers [HXL, HXH, HYL, HYH, HZL, HZH, ST2] at sample rate
    // When reading in "continuous measurement" mode, the ST2 register has to be read at the end.
    i2c_smbus_write_byte_data(fd, R_I2C_SLV0_CTRL, 0b10000111);

    return true;
  }

  bool deinit_driver() override final
  {
    // disable I2C master, clear I2C_MST_EN bit
    i2c_smbus_write_byte_data(fd, R_USER_CTRL, 0x00);
    // enter sleep mode, set SLEEP bit
    i2c_smbus_write_byte_data(fd, R_PWR_MGMT_1, 0b01000000);

    close(fd);

    return true;
  }

  bool read_imu_data() override final
  {
    // read 20 registers with raw values:
    //  - acceleration (3 x 16bit)
    //  - temperature (1 x 16bit)
    //  - gyroscope (3 x 16bit)
    //  - magnetometer (3 x 16bit)
    // note: AK8963 register values are little-endian, while MPU9250 values are big-endian
    uint8_t buffer[20];
    if (i2c_smbus_read_i2c_block_data(fd, R_ACCEL_XOUT_H, sizeof(buffer), buffer) != sizeof(buffer))
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to read IMU data");
      return false;
    }

    // parse and convert raw accelerometer values [g]
    for (int i = 0; i < 3; i++)
    {
      // big-endian
      const int16_t raw = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
      imu.linear_acceleration[i] = raw / accel_sensitivity;
    }

    // parse and convert raw gyroscope values [°/s]
    for (int i = 0; i < 3; i++)
    {
      // big-endian
      const int16_t raw = (buffer[8 + i * 2] << 8) | buffer[8 + i * 2 + 1];
      imu.angular_velocity[i] = raw / gyro_sensitivity;
    }

    // parse and convert raw magnetometer values [µT]
    imu.magnetic_field = {0, 0, 0};
    for (int i = 0; i < 3; i++)
    {
      // little-endian
      const int16_t raw = (buffer[14 + i * 2 + 1] << 8) | buffer[14 + i * 2];
      (*imu.magnetic_field)[i] = raw * mag_sensitivity[i];
    }
    imu.magnetic_field = R_im * (*imu.magnetic_field);

    // parse and convert raw temperature [°C]
    const int16_t raw_temp = (buffer[6] << 8) | buffer[7];
    imu.temperature = raw_temp / temp_sensitivity + 21.0;

    RCLCPP_DEBUG_STREAM(
      get_logger(), "IMU Data:\n"
                      << "  Linear Acceleration [g]: " << std::fixed << std::setprecision(2)
                      << std::setw(8) << std::right << imu.linear_acceleration[0] << std::setw(8)
                      << std::right << imu.linear_acceleration[1] << std::setw(8) << std::right
                      << imu.linear_acceleration[2] << "\n"
                      << "  Angular Velocity [°/s]:  " << std::fixed << std::setprecision(2)
                      << std::setw(8) << std::right << imu.angular_velocity[0] << std::setw(8)
                      << std::right << imu.angular_velocity[1] << std::setw(8) << std::right
                      << imu.angular_velocity[2] << "\n"
                      << "  Magnetic Field [µT]:     " << std::fixed << std::setprecision(2)
                      << std::setw(8) << std::right << (*imu.magnetic_field)[0] << std::setw(8)
                      << std::right << (*imu.magnetic_field)[1] << std::setw(8) << std::right
                      << (*imu.magnetic_field)[2] << "\n"
                      << "  Temperature [°C]:        " << std::fixed << std::setprecision(2)
                      << std::setw(8) << (*imu.temperature));

    return true;
  }

  int fd = 0;

  // addresses
  static constexpr uint8_t A_MPU6050 = 0x68;
  static constexpr uint8_t A_AK8963 = 0x0C;

  // MPU-9250 registers
  static constexpr uint8_t R_SMPLRT_DIV = 0x19;
  static constexpr uint8_t R_CONFIG = 0x1A;
  static constexpr uint8_t R_GYRO_CONFIG = 0x1B;
  static constexpr uint8_t R_ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t R_I2C_MST_CTRL = 0x24;
  static constexpr uint8_t R_I2C_SLV0_ADDR = 0x25;
  static constexpr uint8_t R_I2C_SLV0_REG = 0x26;
  static constexpr uint8_t R_I2C_SLV0_CTRL = 0x27;
  static constexpr uint8_t R_ACCEL_XOUT_H = 0x3B;
  static constexpr uint8_t R_EXT_SENS_DATA_00 = 0x49;
  static constexpr uint8_t R_I2C_SLV0_DO = 0x63;
  static constexpr uint8_t R_USER_CTRL = 0x6A;
  static constexpr uint8_t R_PWR_MGMT_1 = 0x6B;
  static constexpr uint8_t R_WHO_AM_I = 0x75;

  // AK8963 registers
  static constexpr uint8_t R_WIA = 0x00;
  static constexpr uint8_t R_HXL = 0x03;
  static constexpr uint8_t R_CNTL1 = 0x0A;
  static constexpr uint8_t R_CNTL2 = 0x0B;
  static constexpr uint8_t R_ASAX = 0x10;
  static constexpr uint8_t R_ASAY = 0x11;
  static constexpr uint8_t R_ASAZ = 0x12;

  // IDs
  static constexpr uint8_t V_IAM_MPU = 0x71;
  static constexpr uint8_t V_IAM_AK8963 = 0x48;

  // gyroscope scale [°/s]
  // see "MPU-9250 Register Map and Descriptions", Register 27:
  // 0 | ±  250 °/s
  // 1 | ±  500 °/s
  // 2 | ± 1000 °/s
  // 3 | ± 2000 °/s

  // set gyroscope scale to ±500°/s
  static constexpr uint8_t GYRO_FS_SEL = 1;
  static constexpr double gyro_sensitivity = 131 / (1 << GYRO_FS_SEL);  // [LSB/(°/s)]

  // accelerometer scale [g]
  // see "MPU-9250 Register Map and Descriptions", Register 28:
  // 0 | ±  2 g
  // 1 | ±  4 g
  // 2 | ±  8 g
  // 3 | ± 16 g

  // set accelerometer scale to ±4g
  static constexpr uint8_t ACCEL_FS_SEL = 1;
  static constexpr double accel_sensitivity = (2 << 14) / (2 << ACCEL_FS_SEL);  // [LSB/g]

  // magnetometer sensitivity (via the ASA registers)
  std::array<double, 3> mag_sensitivity;  // [µT/LSB]

  // temperature sensor sensitivity
  static constexpr double temp_sensitivity = 333.87;  // [LSB/°C]

  // orientation of magnetometer relative to IMU body frame
  static const Eigen::Matrix3d R_im;
};

// X and Y axes are swapped, Z axis is flipped
const Eigen::Matrix3d MPU9250::R_im = (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();

}  // namespace imu

}  // namespace uav::drivers

PLUGINLIB_EXPORT_CLASS(uav::drivers::imu::MPU9250, hardware_interface::SensorInterface)
