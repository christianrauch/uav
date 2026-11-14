#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <uav_autotune/filter/AlphaFilter.hpp>
#include <uav_autotune/pid_design/pid_design.hpp>
#include <uav_autotune/system_identification/system_identification.hpp>

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <cmath>
#include <numbers>
#include <string>

class UavAutotuneNode : public rclcpp::Node
{
public:
  explicit UavAutotuneNode(const rclcpp::NodeOptions & options)
  : Node("uav_autotune_node", options),
    last_sample_time_(this->now()),
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
    declare_parameter("autotune_enable_topic", "~/autotune_enable");
    declare_parameter("angular_velocity_topic", "~/angular_velocity");
    declare_parameter("autotune_rate_setpoint_topic", "~/autotune_rate_setpoint");
    declare_parameter("status_topic", "~/autotune_status");
  }

  void create_publishers_and_subscriptions()
  {
    rate_setpoint_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("autotune_rate_setpoint_topic").as_string(), 10);
    status_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      get_parameter("status_topic").as_string(), 10);

    angular_velocity_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      get_parameter("angular_velocity_topic").as_string(), 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        angular_velocity_ = *msg;
        angular_velocity_received_ = true;
      });

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("autotune_enable_topic").as_string(), 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        autotune_enabled_ = msg->data;
        if (msg->data && state_ == State::idle)
        {
          state_ = State::init;
          state_start_time_ = now();
        }
      });
  }

  void create_timer()
  {
    const double loop_rate_hz = get_parameter("loop_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz);
    timer_ = create_wall_timer(period, std::bind(&UavAutotuneNode::timer_callback, this));
  }

  void timer_callback()
  {
    const rclcpp::Time now = this->now();

    if (state_ == State::idle && !autotune_enabled_)
    {
      return;
    }

    if (angular_velocity_received_ && last_sample_time_ != rclcpp::Time(0, 0, RCL_ROS_TIME))
    {
      const double dt = (now - last_sample_time_).seconds();
      if (std::isfinite(dt) && dt > 0.0 && dt < 0.02)
      {
        interval_sum_ += static_cast<float>(dt);
        interval_count_ += 1.0f;
      }
    }

    last_sample_time_ = now;
    check_filters();

    const geometry_msgs::msg::Vector3 signal = get_identification_signal();
    rate_setpoint_pub_->publish(signal);

    if (angular_velocity_received_)
    {
      switch (state_)
      {
        case State::roll:
          sys_id_.updateFilters(input_scale_ * signal.x, angular_velocity_.x);
          break;
        case State::pitch:
          sys_id_.updateFilters(input_scale_ * signal.y, angular_velocity_.y);
          break;
        case State::yaw:
          sys_id_.updateFilters(input_scale_ * signal.z, angular_velocity_.z);
          break;
        default:
          break;
      }
    }

    model_update_counter_++;
    if (model_update_counter_ >= model_update_scaler_)
    {
      if (state_ == State::roll || state_ == State::pitch || state_ == State::yaw)
      {
        sys_id_.update();
        last_model_update_ = now;
      }
      model_update_counter_ = 0;
    }

    if ((now - last_publish_).seconds() > get_parameter("state_publish_period_s").as_double() ||
        last_publish_ == rclcpp::Time(0, 0, RCL_ROS_TIME))
    {
      update_state_machine(now);

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

      const Eigen::Vector<float, 5> coeff_var = sys_id_.getVariances();

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
      status_pub_->publish(status);

      if (status.data.size() >= 3)
      {
        RCLCPP_DEBUG_STREAM(
          get_logger(), "fitness=" << status.data[0] << ", kc=" << status.data[5] << ", ki=" <<
            status.data[6] << ", kd=" << status.data[7] << ", att_p=" << status.data[8]);
      }

      (void)coeff_var;
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

  void update_state_machine(const rclcpp::Time & now)
  {
    const float converged_thr = 50.f;

    switch (state_)
    {
      case State::idle:
        if (autotune_enabled_)
        {
          state_ = State::init;
          state_start_time_ = now;
        }
        break;

      case State::init:
        if (filters_initialized_)
        {
          state_ = State::roll;
          state_start_time_ = now;
          sys_id_.reset();
          steps_counter_ = 5;
          max_steps_ = 10;
          signal_sign_ = 1;
          input_scale_ = 1.f / 1.f;
          signal_filter_.reset(0.f);
        }
        break;

      case State::roll:
        if (are_all_smaller_than(sys_id_.getVariances(), converged_thr) &&
            (now - state_start_time_).seconds() > 5.0)
        {
          copy_gains(0);
          state_ = State::roll_pause;
          state_start_time_ = now;
        }
        break;

      case State::roll_pause:
        if ((now - state_start_time_).seconds() > 2.0)
        {
          state_ = State::pitch;
          state_start_time_ = now;
          sys_id_.reset();
          signal_filter_.reset(0.f);
          signal_sign_ = 1;
          steps_counter_ = 5;
          max_steps_ = 10;
        }
        break;

      case State::pitch:
        if (are_all_smaller_than(sys_id_.getVariances(), converged_thr) &&
            (now - state_start_time_).seconds() > 5.0)
        {
          copy_gains(1);
          state_ = State::pitch_pause;
          state_start_time_ = now;
        }
        break;

      case State::pitch_pause:
        if ((now - state_start_time_).seconds() > 2.0)
        {
          state_ = State::yaw;
          state_start_time_ = now;
          sys_id_.reset();
          signal_filter_.reset(0.f);
          signal_sign_ = 1;
          steps_counter_ = 5;
          max_steps_ = 10;
        }
        break;

      case State::yaw:
        if (are_all_smaller_than(sys_id_.getVariances(), converged_thr) &&
            (now - state_start_time_).seconds() > 5.0)
        {
          copy_gains(2);
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
        state_ = are_gains_good() ? State::apply : State::fail;
        state_start_time_ = now;
        break;

      case State::apply:
        state_ = State::complete;
        state_start_time_ = now;
        break;

      case State::test:
        break;

      case State::complete:
      case State::fail:
        if ((now - state_start_time_).seconds() > 2.0)
        {
          state_ = State::idle;
          autotune_enabled_ = false;
        }
        break;

      case State::wait_for_disarm:
        break;
    }
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

  bool are_gains_good() const
  {
    const bool are_positive = rate_k_.minCoeff() > 0.f && rate_i_.minCoeff() > 0.f &&
      rate_d_.minCoeff() >= 0.f && att_p_.minCoeff() > 0.f;
    const bool are_small_enough = rate_k_.maxCoeff() < 0.5f && rate_i_.maxCoeff() < 10.f &&
      rate_d_.maxCoeff() < 0.1f && att_p_.maxCoeff() < 12.f;

    return are_positive && are_small_enough;
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
      case State::complete:
      case State::fail:
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
  bool angular_velocity_received_{false};
  State state_{State::idle};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rate_setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angular_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

  geometry_msgs::msg::Vector3 angular_velocity_{};
  rclcpp::Time last_sample_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_model_update_{0, 0, RCL_ROS_TIME};

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
