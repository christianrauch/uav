// Copyright 2026 Christian Rauch
// Copyright 2021 PAL Robotics SL.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Authors: Christian Rauch, Subhas Das, Denis Stogl, Victor Lopez
 */

#include <memory>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <semantic_components/magnetic_field_sensor.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

namespace magnetometer_broadcaster
{

class MagnetometerBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {
      .type = controller_interface::interface_configuration_type::INDIVIDUAL,
      .names = magnetometer_->get_state_interface_names(),
    };
  }

  controller_interface::CallbackReturn on_init() override { return CallbackReturn::SUCCESS; }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    const std::string sensor_name = get_node()->declare_parameter<std::string>("sensor_name");
    if (sensor_name.empty())
    {
      RCLCPP_FATAL_STREAM(
        get_node()->get_logger(),
        "Parameter 'sensor_name' is empty. Please specify a sensor name for the magnetometer.");
      return controller_interface::CallbackReturn::ERROR;
    }

    const std::string frame_id =
      get_node()->declare_parameter<std::string>("frame_id", std::string{});

    const std::vector<double> static_covariance =
      get_node()->declare_parameter<std::vector<double>>(
        "static_covariance", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    magnetometer_ = std::make_unique<semantic_components::MagneticFieldSensor>(sensor_name);
    try
    {
      sensor_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::MagneticField>(
        "~/magnetic_field", rclcpp::SystemDefaultsQoS());
      realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "Exception thrown during publisher creation at configure stage with message: " << e.what());
      return CallbackReturn::ERROR;
    }

    state_message_.header.frame_id = frame_id;
    std::copy(
      static_covariance.begin(), static_covariance.end(),
      state_message_.magnetic_field_covariance.begin());

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    magnetometer_->assign_loaned_state_interfaces(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    magnetometer_->release_interfaces();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override
  {
    magnetometer_->get_values_as_message(state_message_);

    if (realtime_publisher_)
    {
      state_message_.header.stamp = time;
      realtime_publisher_->try_publish(state_message_);
    }

    return controller_interface::return_type::OK;
  }

protected:
  std::unique_ptr<semantic_components::MagneticFieldSensor> magnetometer_;

  using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr sensor_state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
  sensor_msgs::msg::MagneticField state_message_;
};

}  // namespace magnetometer_broadcaster

PLUGINLIB_EXPORT_CLASS(
  magnetometer_broadcaster::MagnetometerBroadcaster, controller_interface::ControllerInterface)
