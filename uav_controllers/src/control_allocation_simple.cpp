#include <urdf/model.h>
#include <Eigen/Dense>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <uav_controllers/control_allocation_simple_parameters.hpp>

namespace uav::control_allocation
{

class ControlAllocationSimple : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    std::vector<std::string> interface_names(nrotors);
    for (uint8_t i = 0; i < nrotors; i++)
    {
      interface_names[i] = "rotor/" + std::to_string(i + 1) + "/" + "velocity";
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn on_init() override
  {
    // manual control subscriber for reference interface
    sub_control = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/twist", 1,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void { msg_control.set(*msg); });

    param_listener = std::make_shared<ParamListener>(get_node());

    // parse the robot description
    model.initString(get_robot_description());

    const std::string & root = model.getRoot()->name;
    static constexpr std::string_view joint_prefix = "rotor";

    // find all rotary joints connected to the root link
    nrotors = std::count_if(
      model.joints_.cbegin(), model.joints_.cend(),
      [&root](const auto & item)
      {
        const urdf::JointConstSharedPtr joint = item.second;
        return joint->type == urdf::Joint::CONTINUOUS && joint->parent_link_name == root &&
               joint->name.substr(0, joint_prefix.size()) == joint_prefix;
      });

    rotor_pos.setConstant(nrotors, Eigen::NoChange, 0);

    for (uint8_t i = 0; i < nrotors; i++)
    {
      const urdf::JointConstSharedPtr joint =
        model.getJoint(std::string{joint_prefix} + "/" + std::to_string(i + 1));

      if (joint->parent_to_joint_origin_transform.rotation.w != 1)
      {
        // oriented joints are not supported
        RCLCPP_FATAL_STREAM(
          get_node()->get_logger(), "oriented joints ('" << joint->name << "') are not supported");
        return CallbackReturn::ERROR;
      }

      const urdf::Vector3 & p = joint->parent_to_joint_origin_transform.position;
      // rotor direction alternate -1 (CW) / +1 (CCW), starting with +1
      const double dir = std::pow(-1, i + 1);
      rotor_pos.row(i) << p.x, p.y, dir;
    }

    M = compute_M(rotor_pos);

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Motor Mixing Matrix (M): " << std::endl << M);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    min_speed = param_listener->get_params().min_output;

    // set reference interface
    exported_reference_interface_names_.assign(
      reference_interface_names.begin(), reference_interface_names.end());
    reference_interfaces_.resize(exported_reference_interface_names_.size(), 0);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // stop all motors
    bool status_ok = true;
    for (hardware_interface::LoanedCommandInterface & command_interface : command_interfaces_)
    {
      status_ok &= command_interface.set_value(0.);
    }
    return status_ok ? controller_interface::CallbackReturn::SUCCESS
                     : controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (const std::optional<geometry_msgs::msg::Twist> msg = msg_control.try_get())
    {
      reference_interfaces_ = {
        msg->linear.z,   // thrust
        msg->angular.x,  // roll
        msg->angular.y,  // pitch
        msg->angular.z,  // yaw
      };
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    const Eigen::Map<Eigen::Vector4d> control_input(
      reference_interfaces_.data(), reference_interfaces_.size());

    Eigen::ArrayXd w = M * control_input;

    // enforce a minimum motor speed
    // This keeps the thrust direction (controls: roll, pitch, yaw) but increase overall thrust
    // (controls: thrust) to satisfy the individual minimum motor speed.
    Eigen::Index w_min_idx;
    const double w_min = w.minCoeff(&w_min_idx);
    if (w_min < min_speed)
    {
      // desaturation direction
      const Eigen::ArrayXd desat_thrust = M.col(0);

      // move all values along the desaturation vector until the minimum command is reached
      w += (min_speed - w_min) * desat_thrust / desat_thrust[w_min_idx];
    }

    // scale motor command range [min_speed, MAX] to [min_speed, 1] (maxmimum of 1)
    // This reduces thrust direction (controls: roll, pitch, yaw) to reduce individual motor speed.
    const double w_max = w.maxCoeff();
    if (w_max > 1)
    {
      w = min_speed + (w - min_speed) * (1 - min_speed) / (w_max - min_speed);
    }

    // scale all motors by thrust command
    w = min_speed + (w - min_speed) * reference_interfaces_[0];

    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "w [1]: " << w.transpose());

    bool status_ok = true;
    for (std::size_t i = 0; i < nrotors; i++)
    {
      status_ok &= command_interfaces_[i].set_value(w[i]);
    }

    return controller_interface::return_type(!status_ok);
  }

private:
  urdf::Model model;
  uint8_t nrotors = 0;
  Eigen::MatrixX3d rotor_pos;
  Eigen::MatrixX4d M;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_control;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Twist> msg_control;
  std::shared_ptr<ParamListener> param_listener;
  double min_speed = 0;

  static constexpr std::array<std::string, 4> reference_interface_names = {
    "thrust",
    "roll",
    "pitch",
    "yaw",
  };

  static Eigen::MatrixX4d compute_M(const Eigen::MatrixX3d & motors)
  {
    // scale all dimensions by the largest extent
    const double max_size = motors.leftCols<2>().cwiseAbs().maxCoeff();

    // N x [throttle, roll, pitch, yaw]
    Eigen::MatrixX4d M(motors.rows(), Eigen::MatrixX4d::ColsAtCompileTime);

    // base throttle
    M.col(0).setConstant(1);
    // roll from y
    M.col(1) = motors.col(1) / max_size;
    // pitch from x
    M.col(2) = -motors.col(0) / max_size;
    // yaw from direction
    M.col(3) = motors.col(2);

    return M;
  }
};

}  // namespace uav::control_allocation

PLUGINLIB_EXPORT_CLASS(
  uav::control_allocation::ControlAllocationSimple,
  controller_interface::ChainableControllerInterface)
