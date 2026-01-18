#include <imu_filter_madgwick/imu_filter.h>
#include <urdf/model.h>
#include <Eigen/Dense>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace uav::controllers
{

namespace fusion
{

class FusionMadgwick : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> interface_names = {
      // gyroscope
      sensor_name + "/angular_velocity.x",
      sensor_name + "/angular_velocity.y",
      sensor_name + "/angular_velocity.z",
      // accelerometer
      sensor_name + "/linear_acceleration.x",
      sensor_name + "/linear_acceleration.y",
      sensor_name + "/linear_acceleration.z",
    };
    if (use_magnetometer)
    {
      // magnetometer
      interface_names.push_back(sensor_name + "/magnetic_field.x");
      interface_names.push_back(sensor_name + "/magnetic_field.y");
      interface_names.push_back(sensor_name + "/magnetic_field.z");
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    model.initString(get_robot_description());

    use_magnetometer = get_node()->declare_parameter<bool>("use_magnetometer", false);

    exported_state_interface_names_ = {
      // gyroscope
      "angular_velocity.x",
      "angular_velocity.y",
      "angular_velocity.z",
      // accelerometer
      "linear_acceleration.x",
      "linear_acceleration.y",
      "linear_acceleration.z",
      // orientation
      "orientation.w",
      "orientation.x",
      "orientation.y",
      "orientation.z",
    };
    if (use_magnetometer)
    {
      exported_state_interface_names_.insert(
        exported_state_interface_names_.end(), {
                                                 // magnetometer
                                                 "magnetic_field.x",
                                                 "magnetic_field.y",
                                                 "magnetic_field.z",
                                               });
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    sensor_name = get_node()->declare_parameter<std::string>("sensor_name");

    if (sensor_name.empty())
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(),
        "Parameter 'sensor_name' is empty. Please specify a sensor name.");
      return controller_interface::CallbackReturn::ERROR;
    }

    // get rotation between base link and IMU link
    if (const urdf::LinkConstSharedPtr imu_link = model.getLink(sensor_name))
    {
      const std::string & root = model.getRoot()->name;
      if (imu_link->getParent()->name != root)
      {
        RCLCPP_FATAL_STREAM(
          get_node()->get_logger(),
          "IMU link ('" << imu_link->name << "') must be child of root ('" << root << "')!");
        return CallbackReturn::ERROR;
      }

      const urdf::JointConstSharedPtr joint = imu_link->parent_joint;
      if (joint->type != urdf::Joint::FIXED)
      {
        RCLCPP_FATAL_STREAM(
          get_node()->get_logger(), "IMU joint ('" << joint->name << "') must be of type 'FIXED'!");
        return CallbackReturn::ERROR;
      }

      const urdf::Rotation & q = joint->parent_to_joint_origin_transform.rotation;
      q_bi = {
        static_cast<float>(q.w),
        static_cast<float>(q.x),
        static_cast<float>(q.y),
        static_cast<float>(q.z),
      };

      RCLCPP_INFO_STREAM(get_node()->get_logger(), "IMU rotation: " << q_bi.coeffs().transpose());
    }

    filter.setAlgorithmGain(get_node()->declare_parameter<double>("gain", 0.1));

    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool on_set_chained_mode(bool /*chained_mode*/) override { return true; }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    // rotate sensor data from IMU to base frame

    angular_velocity = q_bi * Eigen::Vector3d{
                                state_interfaces_[0].get_optional().value(),
                                state_interfaces_[1].get_optional().value(),
                                state_interfaces_[2].get_optional().value(),
                              };

    linear_acceleration = q_bi * Eigen::Vector3d{
                                   state_interfaces_[3].get_optional().value(),
                                   state_interfaces_[4].get_optional().value(),
                                   state_interfaces_[5].get_optional().value(),
                                 };

    magnetic_field = std::nullopt;
    if (use_magnetometer)
    {
      magnetic_field = q_bi * Eigen::Vector3d{
                                state_interfaces_[6].get_optional().value(),
                                state_interfaces_[7].get_optional().value(),
                                state_interfaces_[8].get_optional().value(),
                              };
    }

    // run the filter update
    if (magnetic_field.has_value())
    {
      filter.madgwickAHRSupdate(
        // gyroscope
        angular_velocity.x(), angular_velocity.y(), angular_velocity.z(),
        // accelerometer
        linear_acceleration.x(), linear_acceleration.y(), linear_acceleration.z(),
        // magnetometer
        magnetic_field->x(), magnetic_field->y(), magnetic_field->z(),
        // sample rate
        period.seconds());
    }
    else
    {
      filter.madgwickAHRSupdateIMU(
        // gyroscope
        angular_velocity.x(), angular_velocity.y(), angular_velocity.z(),
        // accelerometer
        linear_acceleration.x(), linear_acceleration.y(), linear_acceleration.z(),
        // sample rate
        period.seconds());
    }

    // set exported state interfaces

    // gyroscope
    state_interfaces_values_[0] = angular_velocity.x();
    state_interfaces_values_[1] = angular_velocity.y();
    state_interfaces_values_[2] = angular_velocity.z();
    // accelerometer
    state_interfaces_values_[3] = linear_acceleration.x();
    state_interfaces_values_[4] = linear_acceleration.y();
    state_interfaces_values_[5] = linear_acceleration.z();
    // orientation
    filter.getOrientation(
      // quaternion scalar part
      state_interfaces_values_[6],
      // quaternion vector part
      state_interfaces_values_[7], state_interfaces_values_[8], state_interfaces_values_[9]);
    // magnetometer
    if (use_magnetometer && magnetic_field.has_value())
    {
      state_interfaces_values_[10] = magnetic_field->x();
      state_interfaces_values_[11] = magnetic_field->y();
      state_interfaces_values_[12] = magnetic_field->z();
    }

    return controller_interface::return_type::OK;
  }

private:
  std::string sensor_name;
  bool use_magnetometer = false;

  urdf::Model model;
  // rotation from base to IMU frame
  Eigen::Quaterniond q_bi{1, 0, 0, 0};

  // rotated IMU values
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;
  std::optional<Eigen::Vector3d> magnetic_field = std::nullopt;

  ImuFilter filter;
};

}  // namespace fusion

}  // namespace uav::controllers

PLUGINLIB_EXPORT_CLASS(
  uav::controllers::fusion::FusionMadgwick, controller_interface::ChainableControllerInterface)
