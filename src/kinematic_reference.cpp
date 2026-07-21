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

#include "robot_impedance_analyzer/kinematic_reference.hpp"

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

  accelerations_.resize(kSpaceDim, 0);
  velocities_.resize(kSpaceDim, 0);
  positions_.resize(kPoseDim, 0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicReference::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publisher_.reset();
  param_listener_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn KinematicReference::on_activate(
  const rclcpp_lifecycle::State &)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  positions_ = params_.initial_pose;
  accelerations_.assign(kSpaceDim, 0);
  velocities_.assign(kSpaceDim, 0);

  signal_type_ = TypeMap[params_.signal_type];
  axis_ = ::impedance_analysis::AxisMap[*(params_.axis.c_str())];
  uint timer_ms = static_cast<uint>(1000.0 / params_.rate);
  timer_period_ = 1.0 / static_cast<double>(params_.rate);
  start_time_ = this->get_clock()->now();

  switch (signal_type_) {
    case SignalType::kStep:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::step_callback, this));
      break;
    case SignalType::kSineWave:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::sinewave_callback, this));
      break;
    case SignalType::kStepSequence:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::stepseq_callback, this));
      break;
    case SignalType::kCPGLegTrajectory:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::cpg_callback, this));
      break;
    case SignalType::kSquarewave:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::squarewave_callback, this));
      break;
    case SignalType::kSines:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::sines_callback, this));
      break;
    case SignalType::kPRBS:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::prbs_callback, this));
      break;
    case SignalType::kChirp:
      timer_ = this->create_timer(
        std::chrono::milliseconds(timer_ms),
        std::bind(&KinematicReference::chirp_callback, this));
      break;
    default:
      break;
  }

  angular_freq_ = PI_2 / params_.period;

  if (signal_type_ == SignalType::kStepSequence ||
    signal_type_ == SignalType::kCPGLegTrajectory)
  {
    RCLCPP_INFO(get_logger(),
      "Starting '%s' reference signal", params_.signal_type.c_str());
  } else {
    RCLCPP_INFO(get_logger(),
      "Starting '%s' reference signal on axis %s[%lu]",
      params_.signal_type.c_str(),
      params_.axis.c_str(),
      axis_
    );
  }
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

/*
void KinematicReference::publisher_callback()
{
  // Initial pose ('DC' part of the signal)
  for (size_t i = 0; i < kPoseDim; i++) {
    positions_[i] = params_.initial_pose[i];
  }

  switch (signal_type_) {
    case SignalType::kSineWave:
      positions_[axis_] +=
        params_.amplitude * std::sin(angular_freq_ * ellapsed_time_);
      velocities_[axis_] =
        params_.amplitude * angular_freq_ * std::cos(angular_freq_ * ellapsed_time_);
      accelerations_[axis_] =
        -params_.amplitude * std::pow(angular_freq_, 2) * std::sin(angular_freq_ * ellapsed_time_);
      break;
    case SignalType::kCPGLegTrajectory:
      cpg_phase_ = angular_freq_ * ellapsed_time_;
      positions_[0] =
        params_.cpg_x_offset - params_.cpg_length * cpg_amplitude() * std::cos(cpg_phase_);
      if (std::sin(cpg_phase_) > 0.0) {
        positions_[2] = -params_.cpg_robot_height + 0.070 * std::sin(cpg_phase_);
      } else {
        positions_[2] = -params_.cpg_robot_height + 0.009 * std::sin(cpg_phase_);
      }
      break;
    default:
      break;
  }

}
*/

double KinematicReference::cpg_amplitude()
{
  static double ree = 1e-6;

  ree += timer_period_ * (50.0 * (1.0 - ree * ree) * ree);
  return ree;
}

double KinematicReference::lpf_biquad(const double sample)
{
  static double ohm = tanf(M_PI * 0.1);
  static double c = 1.0 + 2 * std::cos(M_PI / 4) * ohm + ohm * ohm;
  static double b0_ = ohm * ohm / c;
  static double b1_ = 2 * b0_;
  static double a1_ = 2 * (ohm * ohm - 1) / c;
  static double a2_ = (1.0 - 2 * std::cos(M_PI / 4) * ohm + ohm * ohm) / c;
  static double u_k1_ = 0;
  static double u_k2_ = 0;
  static double u_k0_;
  static double y_k_;

  u_k0_ = sample - u_k1_ * a1_ - u_k2_ * a2_;
  y_k_ = u_k0_ * b0_ + u_k1_ * b1_ + u_k2_ * b0_;

  u_k2_ = u_k1_;
  u_k1_ = u_k0_;
  return y_k_;
}

void KinematicReference::step_callback()
{
  ellapsed_time_ = (get_clock()->now() - start_time_).seconds();

  positions_[axis_] = params_.initial_pose[axis_];
  positions_[axis_] += ellapsed_time_ > kTimeOffset ? params_.amplitude : 0.0;

  set_message();
  publisher_->publish(message_);
}

void KinematicReference::sinewave_callback()
{
  set_message();
  publisher_->publish(message_);
}

void KinematicReference::stepseq_callback()
{
  ellapsed_time_ = (get_clock()->now() - start_time_).seconds();

  for (size_t k = 0; k < params_.steps.size(); ++k) {
    if (ellapsed_time_ > params_.steps_pose.steps_map[params_.steps[k]].time) {
      for (size_t i = 0; i < kPoseDim; ++i) {
        positions_[i] = params_.steps_pose.steps_map[params_.steps[k]].pose[i];
      }
    }
  }

  set_message();
  publisher_->publish(message_);
}

void KinematicReference::cpg_callback()
{
  set_message();
  publisher_->publish(message_);
}

void KinematicReference::squarewave_callback()
{
  static double phase_ = 0.0;
  static double dphase_ = 0.0;

  dphase_ = 1.0 / (params_.rate * params_.period);  // frequency * dt

  phase_ += dphase_;
  if (phase_ >= 1.0) {phase_ -= 1.0;}  // wrap

  positions_[axis_] = params_.initial_pose[axis_] + params_.amplitude * ((phase_ < 0.5) ? 1 : -1);

  set_message();
  publisher_->publish(message_);
}

void KinematicReference::sines_callback()
{
  static double ang_freq = 0.0;

  ellapsed_time_ = (get_clock()->now() - start_time_).seconds();
  positions_[axis_] = params_.initial_pose[axis_];  // DC component
  accelerations_[axis_] = 0.0;
  velocities_[axis_] = 0.0;

  for (size_t i = 0; i < params_.sines_amp.size(); i++) {
    ang_freq = PI_2 * params_.sines_freq[i];
    positions_[axis_] += params_.sines_amp[i] * std::sin(ang_freq * ellapsed_time_);
    velocities_[axis_] += params_.sines_amp[i] * ang_freq * std::cos(ang_freq * ellapsed_time_);
    accelerations_[axis_] += -params_.sines_amp[i] *
      ang_freq * ang_freq * std::sin(ang_freq * ellapsed_time_);
  }

  set_message();
  publisher_->publish(message_);
}

void KinematicReference::prbs_callback()
{
  static double prbs_settling_time_ = 835;  // 0.835 s
  static double prbs_signal_ = 0.0;
  static uint prbs_counter_ = 0;

  if (prbs_counter_ >= prbs_settling_time_) {
    prbs_signal_ =
      params_.amplitude * static_cast<double>(pseudo_rand()) / pseudo_rand.max();
    prbs_counter_ = 0;
  }
  prbs_counter_++;

  positions_[axis_] = params_.initial_pose[axis_] + lpf_biquad(prbs_signal_);

  set_message();
  publisher_->publish(message_);
}

void KinematicReference::chirp_callback()
{
  static double frequency = 0;
  static double chirp_rate = params_.chirp_final_frequency / params_.chirp_time;

  ellapsed_time_ = (get_clock()->now() - start_time_).seconds();

  frequency = std::min(chirp_rate * ellapsed_time_, params_.chirp_final_frequency);
  positions_[axis_] = params_.initial_pose[axis_] +
    params_.amplitude * std::sin(PI_2 * frequency * ellapsed_time_);
  velocities_[axis_] = 2 * PI_2 * chirp_rate * ellapsed_time_ * params_.amplitude *
    std::cos(PI_2 * frequency * ellapsed_time_);  // chain rule

  set_message();
  publisher_->publish(message_);
}

void KinematicReference::set_message()
{
  message_.pose.position.x = positions_[0];
  message_.pose.position.y = positions_[1];
  message_.pose.position.z = positions_[2];
  // TODO(@me): convert RPY to quaternions
  message_.pose.orientation.x = positions_[3];
  message_.pose.orientation.y = positions_[4];
  message_.pose.orientation.z = positions_[5];
  message_.pose.orientation.w = positions_[6];

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
