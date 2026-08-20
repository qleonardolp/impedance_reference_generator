// Copyright (c) 2026, qleonardolp
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

#include "robot_impedance_analyzer/quadruped_control.hpp"

namespace quadruped_control
{
QuadrupedControl::QuadrupedControl(
  const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

CallbackReturn QuadrupedControl::on_configure(
  const rclcpp_lifecycle::State &)
{
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  auto qos_lowlatency = rclcpp::QoS(1);
  // qos_lowlatency.best_effort().durability_volatile();
  // qos_lowlatency.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);

  fl_publisher_ = create_publisher<KinematicPose>(params_.control_topics[0], qos_lowlatency);
  fr_publisher_ = create_publisher<KinematicPose>(params_.control_topics[1], qos_lowlatency);
  rl_publisher_ = create_publisher<KinematicPose>(params_.control_topics[2], qos_lowlatency);
  rr_publisher_ = create_publisher<KinematicPose>(params_.control_topics[3], qos_lowlatency);

  return CallbackReturn::SUCCESS;
}

CallbackReturn QuadrupedControl::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  fl_publisher_.reset();
  fr_publisher_.reset();
  rl_publisher_.reset();
  rr_publisher_.reset();
  param_listener_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn QuadrupedControl::on_activate(
  const rclcpp_lifecycle::State &)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  uint timer_ms = static_cast<uint>(1000.0 / params_.rate);
  timer_period_ = 1.0 / static_cast<double>(params_.rate);
  start_time_ = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Quadruped control: lifting...");

  set_initial_position();

  double lift_duration = 10.0;  // seconds
  double coeff = (params_.body_height - params_.fl_offset[2]) / lift_duration;
  double lift_time = (get_clock()->now() - start_time_).seconds();

  while (lift_time < lift_duration) {
    lift_time = (get_clock()->now() - start_time_).seconds();
    fl_msg_.pose.position.z = params_.fl_offset[2] + coeff * lift_time;
    fr_msg_.pose.position.z = params_.fr_offset[2] + coeff * lift_time;
    rl_msg_.pose.position.z = params_.rl_offset[2] + coeff * lift_time;
    rr_msg_.pose.position.z = params_.rr_offset[2] + coeff * lift_time;

    publish_references();
  }

  timer_ = this->create_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&QuadrupedControl::gait_callback, this));

  RCLCPP_INFO(get_logger(), "Quadruped control: gait started!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn QuadrupedControl::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  timer_.reset();
  RCLCPP_INFO(get_logger(), "Reference signal stopped");
  return CallbackReturn::SUCCESS;
}

CallbackReturn QuadrupedControl::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    on_cleanup(previous_state);
  }
  return CallbackReturn::SUCCESS;
}

double QuadrupedControl::cpg_amplitude()
{
  static double ree = 1e-6;

  ree += timer_period_ * (50.0 * (1.0 - ree * ree) * ree);
  return ree;
}

void QuadrupedControl::gait_callback()
{
  ellapsed_time_ = (get_clock()->now() - start_time_).seconds();

  publish_references();
}

void QuadrupedControl::set_initial_position()
{
  fl_msg_.pose.position.x = params_.fl_offset[0];
  fl_msg_.pose.position.y = params_.fl_offset[1];
  fl_msg_.pose.position.z = params_.fl_offset[2];

  fr_msg_.pose.position.x = params_.fr_offset[0];
  fr_msg_.pose.position.y = params_.fr_offset[1];
  fr_msg_.pose.position.z = params_.fr_offset[2];

  rl_msg_.pose.position.x = params_.rl_offset[0];
  rl_msg_.pose.position.y = params_.rl_offset[1];
  rl_msg_.pose.position.z = params_.rl_offset[2];

  rr_msg_.pose.position.x = params_.rr_offset[0];
  rr_msg_.pose.position.y = params_.rr_offset[1];
  rr_msg_.pose.position.z = params_.rr_offset[2];

  publish_references();
}

void QuadrupedControl::publish_references()
{
  fl_publisher_->publish(fl_msg_);
  fr_publisher_->publish(fr_msg_);
  rl_publisher_->publish(rl_msg_);
  rr_publisher_->publish(rr_msg_);
}

}  // namespace quadruped_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node =
    std::make_shared<quadruped_control::QuadrupedControl>("quadruped_control");
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
