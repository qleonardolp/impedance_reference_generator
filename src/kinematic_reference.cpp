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

  signal_type_ = TypeMap[params_.signal_type];

  message_ = KinematicPose();

  uint timer_period = static_cast<uint>(1000.0 / params_.rate);

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
  ellapsed_time_ = static_cast<double>(
    (get_clock()->now() - start_time_).nanoseconds()) * 1E-9;

  switch (signal_type_) {
    case SignalType::kStep:
      message_.pose.position.x =
        ellapsed_time_ > kTimeOffset ? params_.amplitude : 0.0;
      break;
    case SignalType::kSmoothStep:
    // Logistic function approximation
      message_.pose.position.x =
        params_.amplitude / (1.0 + std::exp(-200 * (ellapsed_time_ - kTimeOffset)));
      break;
    case SignalType::kSineWave:
      message_.pose.position.x =
        params_.amplitude * std::sin(2 * M_PI / params_.period * ellapsed_time_);
      break;
    case SignalType::kStepUpDown:
    /* code */
      break;
    default:
      break;
  }

  publisher_->publish(message_);
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
