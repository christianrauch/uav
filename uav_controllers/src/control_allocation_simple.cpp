#include <urdf/model.h>
#include <Eigen/Dense>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

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
    // set reference interface
    exported_reference_interface_names_ = {
      "thrust",
      "roll",
      "pitch",
      "yaw",
    };
    reference_interfaces_.resize(exported_reference_interface_names_.size(), 0.0);

    // manual control subscriber for reference interface
    sub_control = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/twist", 1,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void { msg_control.set(*msg); });

    // parse the robot description
    model.initString(get_robot_description());

    const std::string & root = model.getRoot()->name;
    const std::string joint_prefix = "rotor";

    // find all rotary joints connected to the root link
    nrotors = std::count_if(
      model.joints_.cbegin(), model.joints_.cend(),
      [&root, &joint_prefix](const auto & item)
      {
        const urdf::JointConstSharedPtr joint = item.second;
        return joint->type == urdf::Joint::CONTINUOUS && joint->parent_link_name == root &&
               joint->name.substr(0, joint_prefix.size()) == joint_prefix;
      });

    rotor_pos.setConstant(nrotors, Eigen::NoChange, 0);

    for (uint8_t i = 0; i < nrotors; i++)
    {
      const urdf::JointConstSharedPtr joint =
        model.getJoint(joint_prefix + "/" + std::to_string(i + 1));

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
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (const std::optional<geometry_msgs::msg::Twist> msg = msg_control.try_get())
    {
      reference_interfaces_ = {
        msg->linear.z,   // thrust
        msg->angular.y,  // roll
        msg->angular.x,  // pitch
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

    const Eigen::VectorXd w = M * control_input;

    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "w [1]: " << w.transpose());

    bool status_ok = true;
    for (std::size_t i = 0; i < nrotors; i++)
    {
      status_ok &= command_interfaces_[i].set_value(w[i]);
    }

    if (status_ok)
    {
      return controller_interface::return_type::OK;
    }
    else
    {
      return controller_interface::return_type::ERROR;
    }
  }

private:
  urdf::Model model;
  uint8_t nrotors = 0;
  Eigen::MatrixX3d rotor_pos;
  Eigen::MatrixX4d M;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_control;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Twist> msg_control;

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
