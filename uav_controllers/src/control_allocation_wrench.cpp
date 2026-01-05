#include <urdf/model.h>
#include <Eigen/Dense>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <numbers>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>

namespace uav::control_allocation
{

class ControlAllocationWrench : public controller_interface::ChainableControllerInterface
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
      "force.x",  "force.y",  "force.z",   // force
      "torque.x", "torque.y", "torque.z",  // torque
    };
    reference_interfaces_.resize(exported_reference_interface_names_.size(), 0.0);

    // manual control subscriber for reference interface
    sub_control = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
      "~/wrench", 1,
      [this](const geometry_msgs::msg::Wrench::SharedPtr msg) -> void { msg_control.set(*msg); });

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

    rotor_posrot.setConstant(Eigen::NoChange, nrotors, 0);

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

      const urdf::GeometrySharedPtr geometry =
        model.getLink(joint->child_link_name)->visual->geometry;

      if (geometry->type != urdf::Geometry::CYLINDER)
      {
        RCLCPP_FATAL_STREAM(
          get_node()->get_logger(),
          "only cylindrical rotors are supported ('" << joint->name << "')");
        return CallbackReturn::ERROR;
      }

      const double radius = std::static_pointer_cast<urdf::Cylinder>(geometry)->radius;

      if (radius == 0)
      {
        RCLCPP_FATAL_STREAM(
          get_node()->get_logger(), "no rotor radius for joint '" << joint->name << "'");
        return CallbackReturn::ERROR;
      }

      // joint position and rotation axis
      const urdf::Vector3 & p = joint->parent_to_joint_origin_transform.position;
      const urdf::Vector3 & r = joint->axis;

      rotor_posrot.col(i) << p.x, p.y, p.z, r.x, r.y, r.z, radius;

      // normalise rotation axis
      rotor_posrot.col(i).middleRows<3>(3).normalize();
    }

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(), "rotor positions and rotation axes:"
                                  << std::endl
                                  << "(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z)" << std::endl
                                  << rotor_posrot.transpose());

    return controller_interface::CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // thrust/force and torque coefficients [1] (unitless))
    const double cT = get_node()->declare_parameter<double>("cT", 0.05);  // thrust
    const double cQ = get_node()->declare_parameter<double>("cQ", 0.05);  // torque

    // fluid density ρ [kg/m^3]
    // default: 1.225 kg/m^3 (International Standard Atmosphere Layer 0)
    const double p = get_node()->declare_parameter<double>("p", 1.225);

    const Matrix6Xd G = compute_G(rotor_posrot, cT, cQ, p);

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "control allocation matrix (Γ)" << std::endl << G);

    Ginv = G.completeOrthogonalDecomposition().pseudoInverse();

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(), "control allocation matrix (Γ^(-1))" << std::endl
                                                                     << Ginv);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (const std::optional<geometry_msgs::msg::Wrench> msg = msg_control.try_get())
    {
      reference_interfaces_ = {
        msg->force.x,  msg->force.y,  msg->force.z,   // force
        msg->torque.x, msg->torque.y, msg->torque.z,  // torque
      };
    }

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    const Eigen::Map<Eigen::Vector<double, 6>> control_input(
      reference_interfaces_.data(), reference_interfaces_.size());

    const Eigen::ArrayXd w = (Ginv * control_input).array().max(0).sqrt();

    RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "ω [rad/s]: " << w.transpose());

    RCLCPP_DEBUG_STREAM(
      get_node()->get_logger(), "RPM: " << (w * (30 / std::numbers::pi_v<double>)).transpose());

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
  typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixX6d;
  typedef Eigen::Matrix<double, 7, Eigen::Dynamic> Matrix7Xd;

  urdf::Model model;
  uint8_t nrotors = 0;
  Matrix7Xd rotor_posrot;
  MatrixX6d Ginv;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_control;
  realtime_tools::RealtimeThreadSafeBox<geometry_msgs::msg::Wrench> msg_control;

  static Matrix6Xd compute_G(const Matrix7Xd & rotor_pose, double cT, double cQ, double p)
  {
    // matrix Γ mapping from squared motor velocities (ω^2, [rad/s]) to a rigid body wrench:
    // in:
    // - Nx1 rotor velocities squared (ω^2, [rad/s])
    // out:
    // - 3x1 forces (F, [N])
    // - 3x1 torques (τ, [Nm])

    // forces and torques per rotor
    // in terms of revolutions per second n:
    //  T(n) = cT * ρ * n^2 * D^4
    //  Q(n) = cQ * ρ * n^2 * D^5
    // in terms of angular velocity ω:
    //  T(ω) = cT * ρ * D^4 * (2π)^(-2) * ω^2
    //  Q(ω) = cQ * ρ * D^5 * (2π)^(-2) * ω^2
    // where:
    //  T: thrust force [N =  kg⋅m/s^2]
    //  Q: reaction torque [Nm = kg⋅m^2/s^2]
    //  cT: thrust coefficient [1] (unitless)
    //  cQ: torque coefficient [1] (unitless)
    //  ρ: fluid density [kg/m^3]
    //  D: rotor diameter [m]
    //  n: rotor speed [revolutions/s]
    //  ω: rotor angular velocity [rad/s]

    const double p2pi2 = p * std::pow(2 * std::numbers::pi_v<double>, -2);
    const Eigen::ArrayXd r = rotor_pose.bottomRows<1>();
    const Eigen::RowVectorXd kT = cT * p2pi2 * (2 * r).pow(4);  // cT * ρ * (2π)^(-2) * D^4 [kg⋅m]
    const Eigen::RowVectorXd kQ = cQ * p2pi2 * (2 * r).pow(5);  // cQ * ρ * (2π)^(-2) * D^5 [kg⋅m^2]

    // The individual scalar rotor force/torque contributions to the rigid body force/torque are
    // summed up via their position and rotation vectors. This assumes that positions and rotation
    // vectors are given w.r.t. to the centre of mass.
    //  F = sum_i( rot_i * T_i )
    //  M = sum_i( pos_i x (rot_i * T_i) + (-1)^(i+1) * rot_i * Q_i )

    const Eigen::Matrix3Xd pos = rotor_pose.topRows<3>();
    const Eigen::Matrix3Xd rot = rotor_pose.middleRows<3>(3);

    Matrix6Xd G(Matrix6Xd::RowsAtCompileTime, rotor_pose.cols());
    for (int i = 0; i < rotor_pose.cols(); i++)
    {
      // force factor [kg⋅m]
      G.topRows<3>().col(i) = kT(i) * rot.col(i);
      // torque factor [kg⋅m^2]
      G.bottomRows<3>().col(i) =
        pos.col(i).cross(kT(i) * rot.col(i)) + std::pow(-1, i + 1) * kQ(i) * rot.col(i);
    }

    return G;
  }
};

}  // namespace uav::control_allocation

PLUGINLIB_EXPORT_CLASS(
  uav::control_allocation::ControlAllocationWrench,
  controller_interface::ChainableControllerInterface)
