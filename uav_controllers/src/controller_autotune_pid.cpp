#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <format>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uav_controllers/controller_autotune_pid_parameters.hpp>
#include "utils.hpp"

namespace uav::controllers
{

namespace autotune
{

enum class State : uint8_t
{
  IDLE = 0,
  CLIMB,
  STABILIZE_PRE,
  TUNE_ROLL,
  STABILIZE_BETWEEN_1,
  TUNE_PITCH,
  STABILIZE_BETWEEN_2,
  TUNE_YAW,
  COMPLETE
};

struct RLSEstimator
{
  double theta1 = 0.95;
  double theta2 = 0.05;
  double P[2][2] = {{1000.0, 0.0}, {0.0, 1000.0}};
  double lambda = 0.995;
  double prev_w = 0.0;
  double prev_u = 0.0;
  size_t samples = 0;

  void reset()
  {
    theta1 = 0.95;
    theta2 = 0.05;
    P[0][0] = 1000.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 1000.0;
    prev_w = 0.0;
    prev_u = 0.0;
    samples = 0;
  }

  void update(double w_meas, double u_cmd)
  {
    if (samples == 0)
    {
      prev_w = w_meas;
      prev_u = u_cmd;
      samples++;
      return;
    }

    const double phi[2] = {prev_w, prev_u};
    const double y_hat = phi[0] * theta1 + phi[1] * theta2;
    const double error = w_meas - y_hat;

    const double p_phi[2] = {
      P[0][0] * phi[0] + P[0][1] * phi[1], P[1][0] * phi[0] + P[1][1] * phi[1]};

    double denom = lambda + phi[0] * p_phi[0] + phi[1] * p_phi[1];
    if (denom < 1e-7)
    {
      denom = 1e-7;
    }

    const double K[2] = {p_phi[0] / denom, p_phi[1] / denom};

    theta1 += K[0] * error;
    theta2 += K[1] * error;

    // Keep physical constraints
    theta1 = std::clamp(theta1, 0.1, 0.9999);
    if (theta2 < 1e-4)
    {
      theta2 = 1e-4;
    }

    const double P_new[2][2] = {
      {(P[0][0] - K[0] * p_phi[0]) / lambda, (P[0][1] - K[0] * p_phi[1]) / lambda},
      {(P[1][0] - K[1] * p_phi[0]) / lambda, (P[1][1] - K[1] * p_phi[1]) / lambda}};

    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];

    prev_w = w_meas;
    prev_u = u_cmd;
    samples++;
  }

  bool get_continuous_model(double dt, double & a, double & b) const
  {
    if (samples < 50)
    {
      return false;
    }
    const double th1 = std::clamp(theta1, 0.01, 0.99999);
    if (1.0 - th1 < 1e-4)
    {
      a = (1.0 - th1) / dt;
      b = theta2 / dt;
    }
    else
    {
      a = -std::log(th1) / dt;
      b = (theta2 * a) / (1.0 - th1);
    }
    if (std::isnan(a) || std::isnan(b) || b <= 0.0)
    {
      a = 5.0;
      b = std::max(theta2 / dt, 10.0);
    }
    return true;
  }
};

struct IdentifiedGains
{
  double ku = 0.0;
  double tu = 0.0;
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  bool valid = false;
};

class AutotunePID : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {
        std::format("{}/angular_velocity.x", velocity_controller_name),
        std::format("{}/angular_velocity.y", velocity_controller_name),
        std::format("{}/angular_velocity.z", velocity_controller_name),
        std::format("{}/thrust", mixer_name),
      }};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> interface_names = {
      // orientation [quat]
      sensor_name + "/orientation.x",
      sensor_name + "/orientation.y",
      sensor_name + "/orientation.z",
      sensor_name + "/orientation.w",
      // angular velocity [rad/s]
      sensor_name + "/angular_velocity.x",
      sensor_name + "/angular_velocity.y",
      sensor_name + "/angular_velocity.z",
    };

    for (const int64_t channel_id : rpyt_channels)
    {
      interface_names.push_back("rc/channel/" + std::to_string(channel_id));
    }

    if (!pose_sensor_name.empty())
    {
      interface_names.push_back(pose_sensor_name + "/position.x");
      interface_names.push_back(pose_sensor_name + "/position.y");
      interface_names.push_back(pose_sensor_name + "/position.z");
    }

    return {controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

  controller_interface::CallbackReturn on_init() override
  {
    param_listener = std::make_shared<ParamListener>(get_node());

    srv_start = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp)
      {
        std::lock_guard<std::mutex> lock(mutex_state);
        start_autotune_sequence();
        resp->success = true;
        resp->message = "Autotuning sequence started (climbing to safe altitude).";
      });

    srv_stop = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp)
      {
        std::lock_guard<std::mutex> lock(mutex_state);
        current_state = State::IDLE;
        resp->success = true;
        resp->message = "Autotuning stopped.";
      });

    srv_apply = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/apply",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr resp)
      {
        const bool ok = apply_identified_gains();
        resp->success = ok;
        resp->message = ok ? "Successfully applied tuned gains."
                           : "Failed to apply gains (not valid or client error).";
      });

    pub_status = get_node()->create_publisher<std_msgs::msg::String>("~/status", 10);
    pub_rt_status =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(pub_status);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*prev_state*/) override
  {
    const Params params = param_listener->get_params();
    velocity_controller_name = params.velocity_controller_name;
    mixer_name = params.mixer;
    sensor_name = params.sensor_name;
    pose_sensor_name = params.pose_sensor_name;
    rpyt_channels = params.rpyt_channels;

    param_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), velocity_controller_name);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*prev_state*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_state);
    current_state = State::IDLE;
    state_timer = 0.0;
    climb_settle_timer = 0.0;
    integral_z = 0.0;
    filtered_vz = 0.0;
    prev_pos_z_valid = false;
    rls_roll.reset();
    rls_pitch.reset();
    rls_yaw.reset();
    last_cmd_rate = {0.0, 0.0, 0.0};
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*prev_state*/) override
  {
    std::lock_guard<std::mutex> lock(mutex_state);
    current_state = State::IDLE;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    const Params params = param_listener->get_params();
    const double dt = period.seconds();

    // Orientation
    const Eigen::Quaterniond q_state{
      state_interfaces_[3].get_optional().value_or(nan),  // w
      state_interfaces_[0].get_optional().value_or(nan),  // x
      state_interfaces_[1].get_optional().value_or(nan),  // y
      state_interfaces_[2].get_optional().value_or(nan),  // z
    };

    // Angular velocities [rad/s]
    const double wx = state_interfaces_[4].get_optional().value_or(0.0);
    const double wy = state_interfaces_[5].get_optional().value_or(0.0);
    const double wz = state_interfaces_[6].get_optional().value_or(0.0);

    const Eigen::Matrix3d R_state = q_state.toRotationMatrix();
    const double roll_state = std::asin(std::clamp(R_state(2, 1), -1.0, 1.0));
    const double pitch_state = -std::asin(std::clamp(R_state(2, 0), -1.0, 1.0));

    // RC inputs
    const double rc_roll_raw = scale_rpy(state_interfaces_[7]);
    const double rc_pitch_raw = scale_rpy(state_interfaces_[8]);
    const double rc_yaw_raw = -scale_rpy(state_interfaces_[9]);
    const double rc_thrust = scale_t(state_interfaces_[10]);

    // Position Z (Altitude)
    const double pos_z =
      (state_interfaces_.size() > 13) ? state_interfaces_[13].get_optional().value_or(nan) : nan;
    const bool has_position = !std::isnan(pos_z);

    if (has_position)
    {
      const double vz = (prev_pos_z_valid && dt > 0.0) ? (pos_z - prev_pos_z) / dt : 0.0;
      filtered_vz = 0.8 * filtered_vz + 0.2 * vz;
      prev_pos_z = pos_z;
      prev_pos_z_valid = true;
    }

    const double max_tilt_rad = params.max_tilt * std::numbers::pi / 180.0;
    const double target_roll = rc_roll_raw * max_tilt_rad;
    const double target_pitch = rc_pitch_raw * max_tilt_rad;
    const double target_yaw_rate = rc_yaw_raw * 1.5;

    // Safety checks: manual stick override or extreme tilt
    const bool stick_override = (std::abs(rc_roll_raw) > 0.25 || std::abs(rc_pitch_raw) > 0.25);
    const bool tilt_exceeded =
      (std::abs(roll_state) > max_tilt_rad || std::abs(pitch_state) > max_tilt_rad);
    const bool rate_exceeded = (std::abs(wx) > 4.0 || std::abs(wy) > 4.0 || std::abs(wz) > 4.0);

    if (tilt_exceeded || rate_exceeded || stick_override)
    {
      if (current_state != State::IDLE)
      {
        RCLCPP_WARN_STREAM(
          get_node()->get_logger(), "Autotune reset to IDLE due to safety threshold (tilt: "
                                      << tilt_exceeded << ", rate: " << rate_exceeded
                                      << ", stick: " << stick_override << ")");
        current_state = State::IDLE;
      }
    }

    // Baseline attitude stabilization commands
    const double roll_err = target_roll - roll_state;
    const double pitch_err = target_pitch - pitch_state;
    const double yaw_rate_err = target_yaw_rate - wz;

    double cmd_roll_rate =
      params.attitude_gains.roll_p * roll_err + params.attitude_gains.roll_d * wx;
    double cmd_pitch_rate =
      params.attitude_gains.pitch_p * pitch_err + params.attitude_gains.pitch_d * wy;
    double cmd_yaw_rate = params.attitude_gains.yaw_p * yaw_rate_err;

    // Altitude & Thrust Control
    double base_thrust = rc_thrust;
    if (has_position)
    {
      const double target_alt = params.target_altitude;
      const double ez = target_alt - pos_z;

      if (current_state != State::IDLE)
      {
        integral_z = std::clamp(integral_z + ez * dt, -0.2, 0.2);
        base_thrust = params.hover_thrust + params.altitude_gains.p * ez -
                      params.altitude_gains.d * filtered_vz + params.altitude_gains.i * integral_z;
        base_thrust = std::clamp(base_thrust, 0.15, 0.95);
      }
    }

    // Tilt compensation for thrust
    const double cos_tilt = std::max(0.6, std::cos(roll_state) * std::cos(pitch_state));
    const double cmd_thrust = std::clamp(base_thrust / cos_tilt, 0.0, 1.0);

    // State machine for autotuning excitation
    double excitation = 0.0;
    state_timer += dt;

    switch (current_state)
    {
      case State::IDLE:
        break;

      case State::CLIMB:
      {
        // Raise to safe target altitude first
        if (has_position)
        {
          const double alt_err = std::abs(params.target_altitude - pos_z);
          if (alt_err < 0.15 && std::abs(filtered_vz) < 0.2)
          {
            climb_settle_timer += dt;
            if (climb_settle_timer >= 1.0)
            {
              state_timer = 0.0;
              current_state = State::STABILIZE_PRE;
              RCLCPP_INFO_STREAM(
                get_node()->get_logger(),
                "Safe altitude reached (" << pos_z << " m). Pre-stabilizing...");
            }
          }
          else
          {
            climb_settle_timer = 0.0;
          }
        }
        else
        {
          // Fallback if no pose sensor
          if (state_timer >= 2.0)
          {
            state_timer = 0.0;
            current_state = State::STABILIZE_PRE;
          }
        }
        break;
      }

      case State::STABILIZE_PRE:
        if (state_timer >= params.excitation.settle_time)
        {
          state_timer = 0.0;
          rls_roll.reset();
          current_state = State::TUNE_ROLL;
          RCLCPP_INFO(get_node()->get_logger(), "Starting Roll rate autotune excitation...");
        }
        break;

      case State::TUNE_ROLL:
      {
        excitation = generate_chirp(
          state_timer, params.excitation.duration, params.excitation.amplitude_roll,
          params.excitation.freq_min, params.excitation.freq_max);
        cmd_roll_rate += excitation;

        rls_roll.update(wx, last_cmd_rate[0]);

        if (state_timer >= params.excitation.duration)
        {
          identify_axis_gains(rls_roll, dt, params, gains_roll, false);
          state_timer = 0.0;
          current_state = State::STABILIZE_BETWEEN_1;
          RCLCPP_INFO_STREAM(
            get_node()->get_logger(), "Roll autotune complete. Ku: "
                                        << gains_roll.ku << ", Tu: " << gains_roll.tu
                                        << " -> Kp: " << gains_roll.kp << ", Ki: " << gains_roll.ki
                                        << ", Kd: " << gains_roll.kd);
        }
        break;
      }

      case State::STABILIZE_BETWEEN_1:
        if (state_timer >= params.excitation.settle_time)
        {
          state_timer = 0.0;
          rls_pitch.reset();
          current_state = State::TUNE_PITCH;
          RCLCPP_INFO(get_node()->get_logger(), "Starting Pitch rate autotune excitation...");
        }
        break;

      case State::TUNE_PITCH:
      {
        excitation = generate_chirp(
          state_timer, params.excitation.duration, params.excitation.amplitude_pitch,
          params.excitation.freq_min, params.excitation.freq_max);
        cmd_pitch_rate += excitation;

        rls_pitch.update(wy, last_cmd_rate[1]);

        if (state_timer >= params.excitation.duration)
        {
          identify_axis_gains(rls_pitch, dt, params, gains_pitch, false);
          state_timer = 0.0;
          current_state = State::STABILIZE_BETWEEN_2;
          RCLCPP_INFO_STREAM(
            get_node()->get_logger(),
            "Pitch autotune complete. Ku: " << gains_pitch.ku << ", Tu: " << gains_pitch.tu
                                            << " -> Kp: " << gains_pitch.kp << ", Ki: "
                                            << gains_pitch.ki << ", Kd: " << gains_pitch.kd);
        }
        break;
      }

      case State::STABILIZE_BETWEEN_2:
        if (state_timer >= params.excitation.settle_time)
        {
          state_timer = 0.0;
          rls_yaw.reset();
          current_state = State::TUNE_YAW;
          RCLCPP_INFO(get_node()->get_logger(), "Starting Yaw rate autotune excitation...");
        }
        break;

      case State::TUNE_YAW:
      {
        excitation = generate_chirp(
          state_timer, params.excitation.duration, params.excitation.amplitude_yaw,
          params.excitation.freq_min, params.excitation.freq_max);
        cmd_yaw_rate += excitation;

        rls_yaw.update(wz, last_cmd_rate[2]);

        if (state_timer >= params.excitation.duration)
        {
          identify_axis_gains(rls_yaw, dt, params, gains_yaw, true);
          state_timer = 0.0;
          current_state = State::COMPLETE;
          RCLCPP_INFO_STREAM(
            get_node()->get_logger(),
            "Yaw autotune complete. Ku: " << gains_yaw.ku << " -> Kp: " << gains_yaw.kp);
        }
        break;
      }

      case State::COMPLETE:
      {
        if (params.auto_apply)
        {
          apply_identified_gains();
        }
        current_state = State::IDLE;
        break;
      }
    }

    last_cmd_rate = {cmd_roll_rate, cmd_pitch_rate, cmd_yaw_rate};

    // Publish status periodically
    publish_status_telemetry(pos_z);

    // Write commands to hardware
    bool status_ok = true;
    status_ok &= command_interfaces_[0].set_value(cmd_roll_rate);
    status_ok &= command_interfaces_[1].set_value(cmd_pitch_rate);
    status_ok &= command_interfaces_[2].set_value(cmd_yaw_rate);
    status_ok &= command_interfaces_[3].set_value(cmd_thrust);

    return controller_interface::return_type(!status_ok);
  }

private:
  static double scale_rpy(const hardware_interface::LoanedStateInterface & rc_state)
  {
    return (rc_state.get_optional().value_or(0.5) - rc_min) / (rc_max - rc_min) * 2.0 - 1.0;
  }

  static double scale_t(const hardware_interface::LoanedStateInterface & rc_state)
  {
    return (rc_state.get_optional().value_or(0.0) - rc_min) / (rc_max - rc_min);
  }

  static double generate_chirp(
    double t, double duration, double amplitude, double f_min, double f_max)
  {
    if (t >= duration || duration <= 0.0)
    {
      return 0.0;
    }
    const double k = (f_max - f_min) / duration;
    const double phase = 2.0 * std::numbers::pi * (f_min * t + 0.5 * k * t * t);
    // Apply Hann window envelope to ensure smooth start and end
    const double window = 0.5 * (1.0 - std::cos(2.0 * std::numbers::pi * t / duration));
    return amplitude * std::sin(phase) * window;
  }

  void start_autotune_sequence()
  {
    current_state = State::CLIMB;
    state_timer = 0.0;
    climb_settle_timer = 0.0;
    integral_z = 0.0;
    gains_roll.valid = false;
    gains_pitch.valid = false;
    gains_yaw.valid = false;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Initiating autotune: climbing to target altitude before tuning...");
  }

  void identify_axis_gains(
    const RLSEstimator & rls, double dt, const Params & params, IdentifiedGains & gains,
    bool is_yaw)
  {
    double a = 0.0, b = 0.0;
    if (!rls.get_continuous_model(dt, a, b))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "System identification failed to extract continuous model.");
      gains.valid = false;
      return;
    }

    const double tau = params.loop_delay;

    // Solve for ultimate frequency where phase lag = pi: atan(w / a) + w * tau = pi
    double low_w = 1.0;
    double high_w = 2000.0;
    for (int iter = 0; iter < 40; iter++)
    {
      const double mid_w = 0.5 * (low_w + high_w);
      const double phase = std::atan(mid_w / a) + mid_w * tau;
      if (phase < std::numbers::pi)
      {
        low_w = mid_w;
      }
      else
      {
        high_w = mid_w;
      }
    }
    const double w_u = 0.5 * (low_w + high_w);
    const double k_u = std::sqrt(w_u * w_u + a * a) / b;
    const double t_u = 2.0 * std::numbers::pi / w_u;

    gains.ku = k_u;
    gains.tu = t_u;

    if (is_yaw)
    {
      gains.kp = std::clamp(params.tuning_rules.yaw_p_factor * k_u, 0.1, 5.0);
      gains.ki = 0.0;
      gains.kd = 0.0;
    }
    else
    {
      gains.kp = std::clamp(params.tuning_rules.kp_factor * k_u, 0.05, 3.0);
      gains.ki = std::clamp(params.tuning_rules.ki_factor * k_u / t_u, 0.0, 100.0);
      gains.kd = std::clamp(params.tuning_rules.kd_factor * k_u * t_u, 0.00005, 0.05);
    }
    gains.valid = true;
  }

  bool apply_identified_gains()
  {
    if (!param_client)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Parameter client not initialized.");
      return false;
    }

    std::vector<rclcpp::Parameter> new_params;
    if (gains_roll.valid)
    {
      new_params.emplace_back("gains.roll.p", gains_roll.kp);
      new_params.emplace_back("gains.roll.i", gains_roll.ki);
      new_params.emplace_back("gains.roll.d", gains_roll.kd);
    }
    if (gains_pitch.valid)
    {
      new_params.emplace_back("gains.pitch.p", gains_pitch.kp);
      new_params.emplace_back("gains.pitch.i", gains_pitch.ki);
      new_params.emplace_back("gains.pitch.d", gains_pitch.kd);
    }
    if (gains_yaw.valid)
    {
      new_params.emplace_back("gains.yaw.p", gains_yaw.kp);
    }

    if (new_params.empty())
    {
      RCLCPP_WARN(get_node()->get_logger(), "No valid identified gains available to apply.");
      return false;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Applying identified PID parameters to controller...");
    param_client->set_parameters(new_params);
    return true;
  }

  void publish_status_telemetry(double altitude)
  {
    if (pub_rt_status)
    {
      std::string status_str = "STATE: ";
      switch (current_state)
      {
        case State::IDLE:
          status_str += "IDLE";
          break;
        case State::CLIMB:
          status_str += "CLIMB";
          break;
        case State::STABILIZE_PRE:
          status_str += "STABILIZE_PRE";
          break;
        case State::TUNE_ROLL:
          status_str += "TUNE_ROLL";
          break;
        case State::STABILIZE_BETWEEN_1:
          status_str += "STABILIZE_BETWEEN_1";
          break;
        case State::TUNE_PITCH:
          status_str += "TUNE_PITCH";
          break;
        case State::STABILIZE_BETWEEN_2:
          status_str += "STABILIZE_BETWEEN_2";
          break;
        case State::TUNE_YAW:
          status_str += "TUNE_YAW";
          break;
        case State::COMPLETE:
          status_str += "COMPLETE";
          break;
      }

      if (!std::isnan(altitude))
      {
        status_str += std::format(" | Alt: {:.2f}m", altitude);
      }

      if (gains_roll.valid)
      {
        status_str += std::format(
          " | Roll [Kp={:.3f}, Ki={:.3f}, Kd={:.6f}]", gains_roll.kp, gains_roll.ki, gains_roll.kd);
      }
      if (gains_pitch.valid)
      {
        status_str += std::format(
          " | Pitch [Kp={:.3f}, Ki={:.3f}, Kd={:.6f}]", gains_pitch.kp, gains_pitch.ki,
          gains_pitch.kd);
      }
      if (gains_yaw.valid)
      {
        status_str += std::format(" | Yaw [Kp={:.3f}]", gains_yaw.kp);
      }

      std_msgs::msg::String msg;
      msg.data = status_str;
      pub_rt_status->try_publish(msg);
    }
  }

  static constexpr double nan = std::numeric_limits<double>::signaling_NaN();
  static constexpr double rc_min = 0.0;
  static constexpr double rc_max = 1.0;

  std::string velocity_controller_name;
  std::string mixer_name;
  std::string sensor_name;
  std::string pose_sensor_name;
  std::vector<int64_t> rpyt_channels;

  std::shared_ptr<ParamListener> param_listener;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_apply;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>> pub_rt_status;

  std::mutex mutex_state;
  State current_state = State::IDLE;
  double state_timer = 0.0;
  double climb_settle_timer = 0.0;

  double integral_z = 0.0;
  double prev_pos_z = 0.0;
  bool prev_pos_z_valid = false;
  double filtered_vz = 0.0;

  RLSEstimator rls_roll;
  RLSEstimator rls_pitch;
  RLSEstimator rls_yaw;

  std::array<double, 3> last_cmd_rate = {0.0, 0.0, 0.0};

  IdentifiedGains gains_roll;
  IdentifiedGains gains_pitch;
  IdentifiedGains gains_yaw;
};

}  // namespace autotune

}  // namespace uav::controllers

PLUGINLIB_EXPORT_CLASS(
  uav::controllers::autotune::AutotunePID, controller_interface::ControllerInterface)
