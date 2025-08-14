// Copyright (c) 2025, qleonardolp
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

#include "impedance_reference_generator/kinematic_reference.hpp"

#include <limits>

namespace kinematic_reference
{
KinematicReference::KinematicReference(
  const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

CallbackReturn KinematicReference::on_configure(
  const rclcpp_lifecycle::State &)
{
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  auto qos_lowlatency = rclcpp::QoS(1);
  qos_lowlatency.best_effort().durability_volatile();
  qos_lowlatency.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

  publisher_ = create_publisher<KinematicPose>(params_.topic_name, qos_lowlatency);
  power_publisher_ = create_publisher<std_msgs::msg::Float64>(
    "~/step_power", qos_lowlatency);

  accelerations_.resize(kCartesianSpaceDim, 0);
  velocities_.resize(kCartesianSpaceDim, 0);
  positions_.resize(kCartesianSpaceDim, 0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicReference::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publisher_.reset();
  power_publisher_.reset();
  param_listener_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicReference::on_activate(
  const rclcpp_lifecycle::State &)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  angular_freq_ = 2 * M_PI / params_.period;
  signal_type_ = TypeMap[params_.signal_type];
  axis_ = AxisMap[*(params_.axis.c_str())];

  mass_ = params_.mass;
  spring_ = params_.spring;
  damper_ = params_.damper;

  power_last_ = 0.0;
  work_ = 0.0;

  wn_ = std::sqrt(spring_ / mass_);
  zeta_ = damper_ / (2 * std::sqrt(spring_ * mass_));
  zeta_ = std::min(zeta_, 1.000);  // Disallow overdamped systems
  is_critically_damped_ = (1.000 - zeta_) < std::numeric_limits<float>::epsilon();

  if (is_critically_damped_) {
    RCLCPP_WARN(get_logger(),
      "MSD system is critically damped. \u03B6: %.4f, \u03C9: %.4f", zeta_, wn_);
    chi_ = 1.0;
    wd_ = 0.0;
  } else {
    chi_ = std::sqrt(1.0 - zeta_ * zeta_);
    wd_ = wn_ * chi_;
  }
  beta_ = std::atan(zeta_ / chi_);
  sigma_ = wn_ * zeta_;

  accelerations_.assign(kCartesianSpaceDim, 0);
  velocities_.assign(kCartesianSpaceDim, 0);

  message_ = KinematicPose();

  uint timer_period = static_cast<uint>(1000.0 / params_.rate);

  RCLCPP_INFO(get_logger(),
    "Starting '%s' reference signal on axis %s[%lu]",
    params_.signal_type.c_str(),
    params_.axis.c_str(),
    axis_
  );

  start_time_ = this->get_clock()->now();
  timer_ = this->create_timer(
    std::chrono::milliseconds(timer_period),
    std::bind(&KinematicReference::publisher_callback, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicReference::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  RCLCPP_INFO(get_logger(), "Reference signal stopped");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicReference::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    on_cleanup(previous_state);
  }
  return CallbackReturn::SUCCESS;
}

void KinematicReference::publisher_callback()
{
  static double smooth_time = 0.0;

  ellapsed_time_ = static_cast<double>(
    (get_clock()->now() - start_time_).nanoseconds()) * 1E-9;

  smooth_time = ellapsed_time_ - kTimeOffset;

  // Initial pose ('DC' part of the signal)
  positions_[0] = params_.initial_pose[0];
  positions_[1] = params_.initial_pose[1];
  positions_[2] = params_.initial_pose[2];
  positions_[3] = params_.initial_pose[3];
  positions_[4] = params_.initial_pose[4];
  positions_[5] = params_.initial_pose[5];

  switch (signal_type_) {
    case SignalType::kStep:
      positions_[axis_] +=
        ellapsed_time_ > kTimeOffset ? params_.amplitude : 0.0;
      if (ellapsed_time_ > kTimeOffset) {
        step_power(smooth_time);
      }
      break;
    case SignalType::kSmoothStep:
      if (smooth_time < kSmoothStepEnd) {
        positions_[axis_] += logistic_function(smooth_time);
        velocities_[axis_] = logistic_velocity(smooth_time);
        accelerations_[axis_] = logistic_acceleration(smooth_time);
      } else {
        positions_[axis_] += params_.amplitude;
        velocities_[axis_] = 0.0;
        accelerations_[axis_] = 0.0;
      }
      break;
    case SignalType::kSineWave:
      positions_[axis_] +=
        params_.amplitude * std::sin(angular_freq_ * ellapsed_time_);
      velocities_[axis_] =
        params_.amplitude * angular_freq_ * std::cos(angular_freq_ * ellapsed_time_);
      accelerations_[axis_] =
        -params_.amplitude * std::pow(angular_freq_, 2) * std::sin(angular_freq_ * ellapsed_time_);
      break;
    case SignalType::kStepUpDown:
      /* code */
      break;
    default:
      break;
  }

  message_.pose.position.x = positions_[0];
  message_.pose.position.y = positions_[1];
  message_.pose.position.z = positions_[2];
  // TODO(@me): convert RPY to quaternions
  message_.pose.orientation.x = positions_[3];
  message_.pose.orientation.y = positions_[4];
  message_.pose.orientation.z = positions_[5];

  message_.pose_twist.linear.x = velocities_[0];
  message_.pose_twist.linear.y = velocities_[1];
  message_.pose_twist.linear.z = velocities_[2];
  message_.pose_twist.angular.x = velocities_[3];
  message_.pose_twist.angular.y = velocities_[4];
  message_.pose_twist.angular.z = velocities_[5];

  message_.pose_accel.linear.x = accelerations_[0];
  message_.pose_accel.linear.y = accelerations_[1];
  message_.pose_accel.linear.z = accelerations_[2];
  message_.pose_accel.angular.x = accelerations_[3];
  message_.pose_accel.angular.y = accelerations_[4];
  message_.pose_accel.angular.z = accelerations_[5];

  publisher_->publish(message_);
}

double KinematicReference::logistic_function(const double arg)
{
  return params_.amplitude / (1.0 + std::exp(-kSmoothStepSlope * arg));
}

double KinematicReference::logistic_velocity(const double arg)
{
  return params_.amplitude * kSmoothStepSlope *
         std::exp(kSmoothStepSlope * arg) / std::pow(1.0 + std::exp(kSmoothStepSlope * arg), 2);
}

double KinematicReference::logistic_acceleration(const double arg)
{
  static double exp_arg = 0.0;
  static double coeff = 0.0;
  static double num = 0.0;
  static double den = 1.0;

  exp_arg = std::exp(-kSmoothStepSlope * arg);
  coeff = params_.amplitude * kSmoothStepSlope * kSmoothStepSlope;
  num = exp_arg * (exp_arg - 1.0);
  den = std::pow(exp_arg + 1.0, 3);
  return coeff * (num / den);
}

void KinematicReference::step_power(const double time)
{
  static double expt = 0.0;
  static double pos = 0.0;
  static double vel = 0.0;
  static double power = 0.0;

  expt = std::exp(-sigma_ * time);

  if (is_critically_damped_) {  // Ogata, pg. 152
    // Problem: these equations doesn't work well(?) with a
    // critically damped systems (ζ = 1). Maybe for high
    // natural frequency, i.e. high stiffness, a step will
    // generate little to none power, since the displacement
    // is low, thus the velocity is low too.
    pos = 1.0 - expt * (1.0 + wn_ * time);
    vel = wn_ * wn_ * expt * time;
  } else {
    pos = 1.0 - (expt / chi_) * std::cos(wd_ * time - beta_);
    vel = expt * (wn_ / chi_) * std::sin(wd_ * time);
  }

  // Non-unary step scaling
  pos *= params_.amplitude;
  vel *= params_.amplitude;

  power = vel * (spring_ * (params_.amplitude - pos) - damper_ * vel);
  // Trapezoidal integration
  work_ += 0.5 * (power + power_last_) / static_cast<double>(params_.rate);
  power_last_ = power;

  power_.data = work_;
  power_publisher_->publish(power_);
}

}  // namespace kinematic_reference

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<kinematic_reference::KinematicReference>("kinematic_reference");
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
