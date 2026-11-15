#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <control_msgs/msg/pid_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <uav_autotune/filter/AlphaFilter.hpp>
#include <uav_autotune/pid_design/pid_design.hpp>
#include <uav_autotune/system_identification/system_identification.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <mutex>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class UavAutotuneNode : public rclcpp::Node
{
public:
  explicit UavAutotuneNode(const rclcpp::NodeOptions & options)
  : Node("uav_autotune_node", options),
    signal_filter_()
  {
    declare_parameters();
    create_publishers_and_subscriptions();
    create_timer();

    RCLCPP_INFO(get_logger(), "uav_autotune_node started");
  }

private:
  enum class State
  {
    idle,
    init,
    roll,
    roll_pause,
    pitch,
    pitch_pause,
    yaw,
    yaw_pause,
    verification,
    apply,
    test,
    complete,
    fail,
    wait_for_disarm
  };

  void declare_parameters()
  {
    declare_parameter("loop_rate_hz", 200.0);
    declare_parameter("signal_amplitude", 0.3);
    declare_parameter("rise_time", 0.2);
    declare_parameter("state_publish_period_s", 0.1);
    declare_parameter("autotune_enable_service", "~/autotune_enable");
    declare_parameter("angular_velocity_topic", "/fusion_sensor_broadcaster/imu");
    declare_parameter("roll_torque_topic", "/angular_velocity/pid/roll");
    declare_parameter("pitch_torque_topic", "/angular_velocity/pid/pitch");
    declare_parameter("yaw_torque_topic", "/angular_velocity/pid/yaw");
    declare_parameter("roll_attitude_pid_topic", "/attitude/pid/angular_velocity_x");
    declare_parameter("pitch_attitude_pid_topic", "/attitude/pid/angular_velocity_y");
    declare_parameter("yaw_attitude_pid_topic", "/attitude/pid/angular_velocity_z");
    declare_parameter("autotune_rate_setpoint_topic", "/attitude/autotune_rate_setpoint");
    declare_parameter("status_topic", "~/autotune_status");
    declare_parameter("rate_controller_name", "angular_velocity");
    declare_parameter("attitude_controller_name", "attitude");
  }

  void create_publishers_and_subscriptions()
  {
    rate_setpoint_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("autotune_rate_setpoint_topic").as_string(), 10);
    status_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      get_parameter("status_topic").as_string(), 10);

    angular_velocity_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("angular_velocity_topic").as_string(), 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        angular_velocity_ = {
          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        angular_velocity_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
        new_angular_velocity_sample_ = true;
      });

    const std::array<std::string, 3> torque_topics = {
      get_parameter("roll_torque_topic").as_string(),
      get_parameter("pitch_torque_topic").as_string(),
      get_parameter("yaw_torque_topic").as_string()};
    const std::array<std::string, 3> attitude_topics = {
      get_parameter("roll_attitude_pid_topic").as_string(),
      get_parameter("pitch_attitude_pid_topic").as_string(),
      get_parameter("yaw_attitude_pid_topic").as_string()};

    for (size_t i = 0; i < torque_topics.size(); ++i)
    {
      rate_pid_subs_[i] = create_subscription<control_msgs::msg::PidState>(
        torque_topics[i], rclcpp::SystemDefaultsQoS(),
        [this, i](const control_msgs::msg::PidState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(telemetry_mutex_);
          torque_setpoint_[i] = msg->output;
          rate_pid_p_gain_[i] = msg->p_gain;
          rate_pid_i_gain_[i] = msg->i_gain;
          rate_pid_d_gain_[i] = msg->d_gain;
          rate_pid_stamp_[i] = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
          rate_pid_received_[i] = true;
        });

      attitude_pid_subs_[i] = create_subscription<control_msgs::msg::PidState>(
        attitude_topics[i], rclcpp::SystemDefaultsQoS(),
        [this, i](const control_msgs::msg::PidState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(telemetry_mutex_);
          attitude_pid_p_gain_[i] = msg->p_gain;
          attitude_pid_received_[i] = true;
        });
    }

    enable_service_ = create_service<std_srvs::srv::SetBool>(
      get_parameter("autotune_enable_service").as_string(),
      std::bind(&UavAutotuneNode::handle_autotune_enable, this,
        std::placeholders::_1, std::placeholders::_2));

    rate_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, get_parameter("rate_controller_name").as_string());
    attitude_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, get_parameter("attitude_controller_name").as_string());
  }

  void handle_autotune_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      if (state_ != State::idle)
      {
        response->success = false;
        response->message = "Autotune is already active";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }

      last_enable_request_ = false;
      axis_gains_valid_.fill(false);
      {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        autotune_enabled_ = true;
      }
      response->success = true;
      response->message = "Autotune started";
      RCLCPP_INFO(get_logger(), "Autotune enable service requested start");
      return;
    }

    if (state_ == State::apply)
    {
      response->success = false;
      response->message = "Cannot disable autotune while controller gains are being applied";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      autotune_enabled_ = false;
    }
    response->success = true;
    response->message = state_ == State::idle ? "Autotune disabled" : "Autotune abort requested";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  static const char * state_name(State state)
  {
    switch (state)
    {
      case State::idle:
        return "idle";
      case State::init:
        return "initializing";
      case State::roll:
        return "tuning_roll";
      case State::roll_pause:
        return "roll_pause";
      case State::pitch:
        return "tuning_pitch";
      case State::pitch_pause:
        return "pitch_pause";
      case State::yaw:
        return "tuning_yaw";
      case State::yaw_pause:
        return "yaw_pause";
      case State::verification:
        return "verifying";
      case State::apply:
        return "applying_gains";
      case State::test:
        return "testing";
      case State::complete:
        return "complete";
      case State::fail:
        return "failed";
      case State::wait_for_disarm:
        return "waiting_for_disarm";
    }
    return "unknown";
  }

  void create_timer()
  {
    const double loop_rate_hz = get_parameter("loop_rate_hz").as_double();
    if (!std::isfinite(loop_rate_hz) || loop_rate_hz <= 0.0)
    {
      throw std::invalid_argument("loop_rate_hz must be finite and greater than zero");
    }

    const double publish_period_s = get_parameter("state_publish_period_s").as_double();
    if (!std::isfinite(publish_period_s) || publish_period_s <= 0.0)
    {
      throw std::invalid_argument("state_publish_period_s must be finite and greater than zero");
    }

    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz);
    signal_filter_.setParameters(static_cast<float>(publish_period_s), 0.2f);
    timer_ = create_wall_timer(period, std::bind(&UavAutotuneNode::timer_callback, this));
  }

  void timer_callback()
  {
    const rclcpp::Time now = this->now();

    std::array<double, 3> angular_velocity{};
    std::array<double, 3> torque_setpoint{};
    std::array<double, 3> rate_p_gain{};
    std::array<double, 3> rate_i_gain{};
    std::array<double, 3> rate_d_gain{};
    std::array<double, 3> attitude_p_gain{};
    std::array<rclcpp::Time, 3> rate_pid_stamp;
    rclcpp::Time angular_velocity_stamp(0, 0, RCL_ROS_TIME);
    bool new_angular_velocity_sample = false;
    bool autotune_enabled = false;
    bool enable_rising = false;
    std::array<bool, 3> rate_pid_received{};
    std::array<bool, 3> attitude_pid_received{};
    {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      angular_velocity = angular_velocity_;
      torque_setpoint = torque_setpoint_;
      rate_p_gain = rate_pid_p_gain_;
      rate_i_gain = rate_pid_i_gain_;
      rate_d_gain = rate_pid_d_gain_;
      attitude_p_gain = attitude_pid_p_gain_;
      rate_pid_stamp = rate_pid_stamp_;
      rate_pid_received = rate_pid_received_;
      attitude_pid_received = attitude_pid_received_;
      angular_velocity_stamp = angular_velocity_stamp_;
      new_angular_velocity_sample = std::exchange(new_angular_velocity_sample_, false);
      autotune_enabled = autotune_enabled_;
    }

    enable_rising = autotune_enabled && !last_enable_request_;
    last_enable_request_ = autotune_enabled;
    if (state_ == State::idle && !enable_rising)
    {
      return;
    }

    rate_p_gain_ = rate_p_gain;
    rate_i_gain_ = rate_i_gain;
    rate_d_gain_ = rate_d_gain;
    attitude_p_gain_ = attitude_p_gain;

    if (new_angular_velocity_sample && angular_velocity_stamp.nanoseconds() > 0 &&
        std::all_of(rate_pid_received.begin(), rate_pid_received.end(), [](bool received) {
          return received;
        }))
    {
      if (last_sample_time_.nanoseconds() > 0)
      {
        const double measured_dt = (angular_velocity_stamp - last_sample_time_).seconds();
        if (std::isfinite(measured_dt) && measured_dt > 0.0)
        {
          interval_sum_ += static_cast<float>(std::clamp(measured_dt, 0.000125, 0.02));
          interval_count_ += 1.0f;
        }
      }

      last_sample_time_ = angular_velocity_stamp;
      check_filters();

      const int axis = active_axis();
      bool valid_axis_sample = false;
      if (axis >= 0)
      {
        const double pid_age = std::abs((angular_velocity_stamp - rate_pid_stamp[axis]).seconds());
        if (std::isfinite(pid_age) && pid_age <= 0.02)
        {
          sys_id_.updateFilters(
            input_scale_ * static_cast<float>(torque_setpoint[axis]),
            static_cast<float>(angular_velocity[axis]));
          valid_axis_sample = true;
        }
      }

      if (axis < 0 || valid_axis_sample)
      {
        model_update_counter_++;
        if (model_update_counter_ >= model_update_scaler_)
        {
          if (axis >= 0)
          {
            sys_id_.update();
            last_model_update_ = now;
          }
          model_update_counter_ = 0;
        }
      }
    }

    if ((now - last_publish_).seconds() > get_parameter("state_publish_period_s").as_double() ||
        last_publish_ == rclcpp::Time(0, 0, RCL_ROS_TIME))
    {
      update_state_machine(now, autotune_enabled, enable_rising, attitude_pid_received);

      const Eigen::Vector<float, 5> coeff = sys_id_.getCoefficients();
      Eigen::Vector<float, 5> coeff_scaled = coeff;
      coeff_scaled(2) *= input_scale_;
      coeff_scaled(3) *= input_scale_;
      coeff_scaled(4) *= input_scale_;

      const Eigen::Vector3f num(coeff_scaled(2), coeff_scaled(3), coeff_scaled(4));
      const Eigen::Vector3f den(1.f, coeff_scaled(0), coeff_scaled(1));
      const float model_dt = static_cast<float>(model_update_scaler_) * filter_dt_;
      const float desired_rise_time = (state_ == State::yaw || state_ == State::yaw_pause)
        ? 0.2f
        : static_cast<float>(get_parameter("rise_time").as_double());
      Eigen::Vector3f kid = pid_design::computePidGmvc(num, den, model_dt, desired_rise_time, 0.f, 0.7f);

      if ((kid(2) < 0.f) && (kid(2) > -0.001f))
      {
        kid(2) = 0.f;
      }

      const float attitude_p = std::clamp(1.f / (std::numbers::pi_v<float> / 180.f * 60.f * kid(0)), 2.f, 6.5f);
      kid_ = kid;
      attitude_p_ = attitude_p;

      const Eigen::Vector<float, 5> coeff_var = sys_id_.getVariances();
      const geometry_msgs::msg::Vector3 rate_sp =
        sys_id_.areFiltersInitialized() ? get_identification_signal() : geometry_msgs::msg::Vector3{};
      rate_setpoint_pub_->publish(rate_sp);

      std_msgs::msg::Float32MultiArray status;
      status.data = {
        sys_id_.getFitness(),
        model_dt,
        sys_id_.getInnovation(),
        sys_id_.getFilteredInputData(),
        sys_id_.getFilteredOutputData(),
        kid(0),
        kid(1),
        kid(2),
        attitude_p,
        static_cast<float>(static_cast<int>(state_))};
      status.data.insert(status.data.end(), coeff_scaled.data(), coeff_scaled.data() + 5);
      status.data.insert(status.data.end(), coeff_var.data(), coeff_var.data() + 5);
      status.data.insert(status.data.end(), {
        static_cast<float>(rate_sp.x), static_cast<float>(rate_sp.y),
        static_cast<float>(rate_sp.z)});
      status_pub_->publish(status);

      if (status.data.size() >= 3)
      {
        RCLCPP_DEBUG_STREAM(
          get_logger(), "state=" << state_name(state_) << ", fitness=" << status.data[0] <<
            ", innovation=" << status.data[2] << ", input=" << status.data[3] << ", output=" <<
            status.data[4] << ", kc=" << status.data[5] << ", ki=" << status.data[6] <<
            ", kd=" << status.data[7] << ", att_p=" << status.data[8]);
      }

      last_publish_ = now;
    }
  }

  void check_filters()
  {
    if (interval_count_ > 1000.0f)
    {
      sample_interval_avg_ = interval_sum_ / interval_count_;
      bool reset_filters = false;

      const float filter_dt = filter_dt_;
      if (std::fabs(filter_dt - sample_interval_avg_) / std::max(filter_dt, 1e-6f) > 0.01f)
      {
        reset_filters = true;
      }

      if (reset_filters || !filters_initialized_)
      {
        filter_dt_ = sample_interval_avg_;
        const float filter_rate_hz = 1.f / filter_dt_;

        sys_id_.setLpfCutoffFrequency(filter_rate_hz, 30.f);
        sys_id_.setHpfCutoffFrequency(filter_rate_hz, 0.5f);

        float model_dt = std::clamp(std::max(1.f / (2.f * 30.f), filter_dt_), 2e-3f, 10e-3f);
        model_update_scaler_ = std::max(static_cast<int>(model_dt / filter_dt_), 1);
        model_dt = static_cast<float>(model_update_scaler_) * filter_dt_;

        sys_id_.setForgettingFactor(60.f, model_dt);
        sys_id_.setFitnessLpfTimeConstant(1.f, model_dt);
        filters_initialized_ = true;
      }

      last_sample_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      interval_sum_ = 0.f;
      interval_count_ = 0.f;
    }
  }

  void update_state_machine(
    const rclcpp::Time & now, bool enable_requested, bool enable_rising,
    const std::array<bool, 3> & attitude_pid_received)
  {
    const State previous_state = state_;
    const float converged_thr = 50.f;

    if (
      !enable_requested && state_ != State::idle && state_ != State::complete &&
      state_ != State::fail && state_ != State::apply)
    {
      state_ = State::fail;
      state_start_time_ = now;
      RCLCPP_WARN(get_logger(), "Autotune aborted by disable request");
    }

    switch (state_)
    {
      case State::idle:
        if (enable_rising)
        {
          state_ = State::init;
          state_start_time_ = now;
          gain_update_started_ = false;
        }
        break;

      case State::init:
        if (filters_initialized_)
        {
          if (set_input_scale(0))
          {
            state_ = State::roll;
            state_start_time_ = now;
            sys_id_.reset();
            steps_counter_ = 5;
            max_steps_ = 10;
            signal_sign_ = 1;
            signal_filter_.reset(0.f);
          }
          else
          {
            fail_autotune(now, "Roll tuning failed: rate PID P gain or normalization is invalid");
          }
        }
        break;

      case State::roll:
        if (are_all_smaller_than(sys_id_.getVariances(), converged_thr) &&
            (now - state_start_time_).seconds() > 5.0)
        {
          copy_gains(0);
          validate_and_report_axis_gains(0);
          state_ = State::roll_pause;
          state_start_time_ = now;
        }
        break;

      case State::roll_pause:
        if ((now - state_start_time_).seconds() > 2.0)
        {
          if (set_input_scale(1))
          {
            state_ = State::pitch;
            state_start_time_ = now;
            sys_id_.reset();
            signal_filter_.reset(0.f);
            signal_sign_ = 1;
            steps_counter_ = 5;
            max_steps_ = 10;
          }
          else
          {
            fail_autotune(now, "Pitch tuning failed: rate PID P gain or normalization is invalid");
          }
        }
        break;

      case State::pitch:
        if (are_all_smaller_than(sys_id_.getVariances(), converged_thr) &&
            (now - state_start_time_).seconds() > 5.0)
        {
          copy_gains(1);
          validate_and_report_axis_gains(1);
          state_ = State::pitch_pause;
          state_start_time_ = now;
        }
        break;

      case State::pitch_pause:
        if ((now - state_start_time_).seconds() > 2.0)
        {
          if (set_input_scale(2))
          {
            state_ = State::yaw;
            state_start_time_ = now;
            sys_id_.reset();
            signal_filter_.reset(0.f);
            signal_sign_ = 1;
            steps_counter_ = 5;
            max_steps_ = 10;
          }
          else
          {
            fail_autotune(now, "Yaw tuning failed: rate PID P gain or normalization is invalid");
          }
        }
        break;

      case State::yaw:
        if (are_all_smaller_than(sys_id_.getVariances(), converged_thr) &&
            (now - state_start_time_).seconds() > 5.0)
        {
          copy_gains(2);
          validate_and_report_axis_gains(2);
          state_ = State::yaw_pause;
          state_start_time_ = now;
        }
        break;

      case State::yaw_pause:
        if ((now - state_start_time_).seconds() > 2.0)
        {
          state_ = State::verification;
          state_start_time_ = now;
          sys_id_.reset();
          signal_filter_.reset(0.f);
          signal_sign_ = 1;
          steps_counter_ = 5;
          max_steps_ = 10;
        }
        break;

      case State::verification:
        state_ = std::all_of(
            axis_gains_valid_.begin(), axis_gains_valid_.end(), [](bool valid) { return valid; }) ?
          State::apply :
          State::fail;
        state_start_time_ = now;
        break;

      case State::apply:
        if ((now - state_start_time_).seconds() > 20.0)
        {
          fail_autotune(now, "Timed out applying gains");
        }
        else
        {
          update_gain_application(now, attitude_pid_received);
        }
        break;

      case State::test:
        break;

      case State::complete:
      case State::fail:
        if ((now - state_start_time_).seconds() > 2.0)
        {
          state_ = State::idle;
          gain_update_started_ = false;
          rollback_pending_ = false;
        }
        break;

      case State::wait_for_disarm:
        break;
    }

    if (
      state_ != State::idle && state_ != State::complete && state_ != State::fail &&
      state_ != State::apply && (now - state_start_time_).seconds() > 20.0)
    {
      const int axis = active_axis();
      const std::string reason = axis >= 0 ?
        std::string("Timed out waiting for ") + axis_name(static_cast<size_t>(axis)) +
          " system identification to converge" :
        "Timed out waiting for system identification to converge";
      fail_autotune(now, reason);
    }

    if (state_ != previous_state)
    {
      RCLCPP_DEBUG(
        get_logger(), "Autotune state changed: %s -> %s", state_name(previous_state),
        state_name(state_));
    }
  }

  int active_axis() const
  {
    switch (state_)
    {
      case State::roll:
        return 0;
      case State::pitch:
        return 1;
      case State::yaw:
        return 2;
      default:
        return -1;
    }
  }

  bool set_input_scale(size_t axis)
  {
    const double p_gain = rate_p_gain_[axis];
    if (!std::isfinite(p_gain) || p_gain <= 0.0)
    {
      return false;
    }

    input_scale_ = static_cast<float>(1.0 / p_gain);
    return std::isfinite(input_scale_);
  }

  void fail_autotune(const rclcpp::Time & now, const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    state_ = State::fail;
    state_start_time_ = now;
  }

  std::vector<rclcpp::Parameter> make_rate_gain_parameters(
    const std::array<double, 3> & p, const std::array<double, 3> & i,
    const std::array<double, 3> & d) const
  {
    const std::array<std::string, 3> axes = {"roll", "pitch", "yaw"};
    std::vector<rclcpp::Parameter> parameters;
    parameters.reserve(9);
    for (size_t axis = 0; axis < axes.size(); ++axis)
    {
      const std::string prefix = "gains." + axes[axis] + ".";
      parameters.emplace_back(prefix + "p", p[axis]);
      parameters.emplace_back(prefix + "i", i[axis]);
      parameters.emplace_back(prefix + "d", d[axis]);
    }
    return parameters;
  }

  std::vector<rclcpp::Parameter> make_attitude_gain_parameters(
    const std::array<double, 3> & gains) const
  {
    return {
      rclcpp::Parameter("gains.roll.p", gains[0]),
      rclcpp::Parameter("gains.pitch.p", gains[1]),
      rclcpp::Parameter("gains.yaw_rate.p", gains[2])};
  }

  void update_gain_application(
    const rclcpp::Time & now, const std::array<bool, 3> & attitude_pid_received)
  {
    using namespace std::chrono_literals;
    if (!gain_update_started_)
    {
      if (!std::all_of(attitude_pid_received.begin(), attitude_pid_received.end(), [](bool value) {
            return value;
          }))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Waiting for all attitude PID telemetry before applying gains");
        return;
      }

      if (!rate_parameters_client_->service_is_ready() ||
          !attitude_parameters_client_->service_is_ready())
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Waiting for controller parameter services");
        return;
      }

      original_rate_p_ = rate_pid_p_gain_;
      original_rate_i_ = rate_pid_i_gain_;
      original_rate_d_ = rate_pid_d_gain_;
      original_attitude_p_ = attitude_p_gain_;

      std::array<double, 3> new_rate_p{};
      std::array<double, 3> new_rate_i{};
      std::array<double, 3> new_rate_d{};
      for (size_t axis = 0; axis < new_rate_p.size(); ++axis)
      {
        new_rate_p[axis] = rate_k_(axis);
        new_rate_i[axis] = rate_k_(axis) * rate_i_(axis);
        new_rate_d[axis] = rate_k_(axis) * rate_d_(axis);
      }

      rate_gain_future_ = rate_parameters_client_->set_parameters_atomically(
        make_rate_gain_parameters(new_rate_p, new_rate_i, new_rate_d));
      attitude_gain_future_ = attitude_parameters_client_->set_parameters_atomically(
        make_attitude_gain_parameters({
          att_p_(0), att_p_(1), att_p_(2)}));
      gain_update_started_ = true;
      return;
    }

    if (rollback_pending_)
    {
      if (rollback_future_.wait_for(0s) != std::future_status::ready)
      {
        return;
      }

      const auto result = rollback_future_.get();
      if (!result.successful)
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to roll back partial gain update: %s", result.reason.c_str());
      }
      fail_autotune(now, "Controller gain update was only partially successful");
      return;
    }

    if (
      rate_gain_future_.wait_for(0s) != std::future_status::ready ||
      attitude_gain_future_.wait_for(0s) != std::future_status::ready)
    {
      return;
    }

    const auto rate_result = rate_gain_future_.get();
    const auto attitude_result = attitude_gain_future_.get();
    if (rate_result.successful && attitude_result.successful)
    {
      state_ = State::complete;
      state_start_time_ = now;
      RCLCPP_INFO(get_logger(), "Autotune gains applied to both controllers");
      return;
    }

    if (!rate_result.successful)
    {
      RCLCPP_ERROR(get_logger(), "Rate gain update failed: %s", rate_result.reason.c_str());
    }
    if (!attitude_result.successful)
    {
      RCLCPP_ERROR(get_logger(), "Attitude gain update failed: %s", attitude_result.reason.c_str());
    }

    if (rate_result.successful != attitude_result.successful)
    {
      rollback_future_ = rate_result.successful
        ? rate_parameters_client_->set_parameters_atomically(
            make_rate_gain_parameters(original_rate_p_, original_rate_i_, original_rate_d_))
        : attitude_parameters_client_->set_parameters_atomically(
            make_attitude_gain_parameters(original_attitude_p_));
      rollback_pending_ = true;
      return;
    }

    fail_autotune(now, "Controller gain update failed");
  }

  bool are_all_smaller_than(const Eigen::Vector<float, 5> & vect, float threshold) const
  {
    return vect(0) < threshold && vect(1) < threshold && vect(2) < threshold &&
           vect(3) < threshold && vect(4) < threshold;
  }

  void copy_gains(int index)
  {
    if (index <= 2)
    {
      rate_k_(index) = kid_(0);
      rate_i_(index) = kid_(1);
      rate_d_(index) = kid_(2);
      att_p_(index) = attitude_p_;
    }
  }

  static const char * axis_name(size_t axis)
  {
    static constexpr std::array<const char *, 3> names{"Roll", "Pitch", "Yaw"};
    return names[axis];
  }

  std::string validate_axis_gains(size_t axis) const
  {
    static constexpr std::array<const char *, 4> names{
      "rate Kc", "rate Ki", "rate Kd", "attitude P"};
    static constexpr std::array<float, 4> upper_limits{0.5f, 10.f, 0.1f, 12.f};
    const std::array<float, 4> values{
      rate_k_(axis), rate_i_(axis), rate_d_(axis), att_p_(axis)};

    for (size_t gain = 0; gain < values.size(); ++gain)
    {
      const std::string label = std::string(axis_name(axis)) + " tuning failed: " + names[gain];
      if (!std::isfinite(values[gain]))
      {
        return label + " is not finite (got " + std::to_string(values[gain]) + ")";
      }

      const bool lower_bound_valid = gain == 2 ? values[gain] >= 0.f : values[gain] > 0.f;
      if (!lower_bound_valid || values[gain] >= upper_limits[gain])
      {
        const char * lower_bound = gain == 2 ? "[0, " : "(0, ";
        return label + "=" + std::to_string(values[gain]) + " must be in " + lower_bound +
               std::to_string(upper_limits[gain]) + ")";
      }
    }

    return {};
  }

  void validate_and_report_axis_gains(size_t axis)
  {
    const std::string error = validate_axis_gains(axis);
    if (!error.empty())
    {
      axis_gains_valid_[axis] = false;
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return;
    }

    axis_gains_valid_[axis] = true;
    RCLCPP_INFO(get_logger(), "%s axis candidate gains passed validation", axis_name(axis));
  }

  geometry_msgs::msg::Vector3 get_identification_signal()
  {
    if (steps_counter_ > max_steps_)
    {
      signal_sign_ = (signal_sign_ == 1) ? 0 : 1;
      steps_counter_ = 0;
      if (max_steps_ > 1)
      {
        max_steps_--;
      }
      else
      {
        max_steps_ = 5;
      }
    }

    steps_counter_++;
    const float step = static_cast<float>(signal_sign_) * static_cast<float>(get_parameter("signal_amplitude").as_double());

    geometry_msgs::msg::Vector3 rate_sp{};
    const float signal = step - signal_filter_.getState();

    switch (state_)
    {
      case State::roll:
        rate_sp.x = signal;
        break;
      case State::pitch:
        rate_sp.y = signal;
        break;
      case State::yaw:
        rate_sp.z = signal;
        break;
      case State::test:
        rate_sp.x = signal;
        rate_sp.y = signal;
        break;
      default:
        break;
    }

    signal_filter_.update(step);
    return rate_sp;
  }

  bool autotune_enabled_{false};
  bool last_enable_request_{false};
  std::array<bool, 3> axis_gains_valid_{};
  bool new_angular_velocity_sample_{false};
  State state_{State::idle};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rate_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr angular_velocity_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  std::array<rclcpp::Subscription<control_msgs::msg::PidState>::SharedPtr, 3> rate_pid_subs_;
  std::array<rclcpp::Subscription<control_msgs::msg::PidState>::SharedPtr, 3>
    attitude_pid_subs_;
  std::shared_ptr<rclcpp::AsyncParametersClient> rate_parameters_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> attitude_parameters_client_;

  std::mutex telemetry_mutex_;
  std::array<double, 3> angular_velocity_{};
  std::array<double, 3> torque_setpoint_{};
  std::array<double, 3> rate_pid_p_gain_{};
  std::array<double, 3> rate_pid_i_gain_{};
  std::array<double, 3> rate_pid_d_gain_{};
  std::array<double, 3> attitude_pid_p_gain_{};
  std::array<rclcpp::Time, 3> rate_pid_stamp_{
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(0, 0, RCL_ROS_TIME),
    rclcpp::Time(0, 0, RCL_ROS_TIME)};
  std::array<bool, 3> rate_pid_received_{};
  std::array<bool, 3> attitude_pid_received_{};
  rclcpp::Time angular_velocity_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sample_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_model_update_{0, 0, RCL_ROS_TIME};

  std::array<double, 3> rate_p_gain_{};
  std::array<double, 3> rate_i_gain_{};
  std::array<double, 3> rate_d_gain_{};
  std::array<double, 3> attitude_p_gain_{};
  std::array<double, 3> original_rate_p_{};
  std::array<double, 3> original_rate_i_{};
  std::array<double, 3> original_rate_d_{};
  std::array<double, 3> original_attitude_p_{};
  std::shared_future<rcl_interfaces::msg::SetParametersResult> rate_gain_future_;
  std::shared_future<rcl_interfaces::msg::SetParametersResult> attitude_gain_future_;
  std::shared_future<rcl_interfaces::msg::SetParametersResult> rollback_future_;
  bool gain_update_started_{false};
  bool rollback_pending_{false};

  float interval_sum_{0.f};
  float interval_count_{0.f};
  float sample_interval_avg_{0.01f};
  float filter_dt_{0.01f};
  bool filters_initialized_{false};
  SystemIdentification sys_id_{};
  AlphaFilter<float> signal_filter_;

  float input_scale_{1.f};
  int model_update_scaler_{1};
  int model_update_counter_{0};
  int steps_counter_{0};
  int max_steps_{5};
  int signal_sign_{0};

  Eigen::Vector3f rate_k_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f rate_i_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f rate_d_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f att_p_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f kid_{Eigen::Vector3f::Zero()};
  float attitude_p_{0.f};
};

RCLCPP_COMPONENTS_REGISTER_NODE(UavAutotuneNode)
