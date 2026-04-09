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

#include "robot_impedance_analyzer/impedance_id.hpp"

namespace impedance_identification
{
ImpedanceId::ImpedanceId(
  const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  configure();  // self configure
}

CallbackReturn ImpedanceId::on_configure(
  const rclcpp_lifecycle::State &)
{
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  estimates_.data.resize(8, 0.0);
  param_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("~/estimated_params", 1);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceId::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  param_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceId::on_activate(
  const rclcpp_lifecycle::State &)
{
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  axis_ = ::impedance_analysis::AxisMap[*(params_.axis.c_str())];

  double beta = 2 * M_PI * params_.cutoff_frequency / params_.sampling_frequency;
  lpf_alpha_ = beta / (beta + 1);

  /* RLS */
  phi_.setOnes();
  error_.setZero();
  theta_.setZero();
  cov_k_.setIdentity();
  cov_k_ *= 10'000;
  rls_gain_den_ = 1.0;

  theta_(0) = 1.0;
  theta_last_ = theta_;

  k_m_ratio_ = params_.expected_stiffness / params_.expected_mass;
  d_m_ratio_ = params_.expected_damping / params_.expected_mass;

  /* ISPI */
  point_counter_ = 0;
  plane_n_ <<
    Eigen::Vector3d(
      params_.expected_stiffness,
      params_.expected_damping,
      params_.expected_mass).normalized();
  plane_n_last_ = plane_n_;
  plane_n_filt_ = plane_n_;

  output_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    params_.controller_status_topic, rclcpp::QoS(1).best_effort(),
    std::bind(&ImpedanceId::output_callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(
    get_logger(), "Running 'Z' identification on axis %s.", params_.axis.c_str());
  last_clock_ = get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceId::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  output_subscriber_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceId::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    on_cleanup(previous_state);
  }
  return CallbackReturn::SUCCESS;
}

void ImpedanceId::output_callback(const std_msgs::msg::Float64MultiArray & status_msg)
{
  delta_t_ = 1E-9 * static_cast<double>((get_clock()->now() - last_clock_).nanoseconds());

  new_output_(0) = status_msg.data[kDevId + axis_];  // deviation
  new_output_(1) = status_msg.data[kVelId + axis_];  // deviation derivative
  new_output_(2) = status_msg.data[kAccId + axis_];  // deviation 2nd derivative

  step_detected_ =
    abs(status_msg.data[kDevId + axis_] - new_point_(0)) > kPosDeltaThreshold;
  estimates_.data[7] = step_detected_;

  if (!step_detected_) {
    new_point_(0) = status_msg.data[kDevId + axis_];
    new_point_(1) = status_msg.data[kVelId + axis_];
    new_point_(2) = status_msg.data[kAccId + axis_];
  }

  update_ispi();
  update_rls();
  param_publisher_->publish(estimates_);
  last_clock_ = get_clock()->now();
}

void ImpedanceId::update_rls()
{
  // Update the regression vector
  phi_(0) = k_m_ratio_ * new_output_(0) + d_m_ratio_ * new_output_(1);
  phi_(0) = - phi_(0);
  phi_(1) = 1.0 / params_.expected_mass;

  // Update Gain
  rls_gain_den_ = lambda_ + phi_.transpose() * cov_k_ * phi_;
  gain_k_.noalias() = (cov_k_ * phi_) / rls_gain_den_;

  // Update error
  error_.noalias() = new_output_.tail<kSpaceDim>() - theta_.transpose() * phi_;

  // New estimation
  theta_ = theta_ + gain_k_ * error_.transpose();
  // New covariance
  cov_k_ = (CovarianceMatrix::Identity() - gain_k_ * phi_.transpose()) * cov_k_ / lambda_;

  estimates_.data[5] = theta_(0);  // s
  estimates_.data[6] = theta_(1);  // l
  estimates_.data[7] = error_(0);  // regression error
}

void ImpedanceId::update_ispi()
{
  // Point counter FSM
  if (point_counter_ == 2) {
    direction_v2_ = new_point_ - first_point_;
    ++point_counter_;  // -> 3
  }

  if (point_counter_ == 1) {
    direction_v1_ = new_point_ - first_point_;
    ++point_counter_;  // -> 2
  }

  if (point_counter_ == 0) {
    first_point_ = new_point_;
    ++point_counter_;  // -> 1
  }

  if (0 == point_counter_ % 3) {
    // Computes only every three new points.
    // This is to decouple the effect of d (`f_int`)
    // between plane estimations.
    cross_prod_ = direction_v2_.cross(direction_v1_);
    cluster_area_ = cross_prod_.norm();
    if (cluster_area_ > kAreaThreshold) {
      plane_n_ = cross_prod_.normalized();
    }
    // Fix sign flipping
    if (plane_n_.dot(plane_n_last_) < 0) {
      plane_n_ = -plane_n_;
    }
    plane_n_last_ = plane_n_;

    // sample and hold first_point_
    first_last_ = first_point_;

    point_counter_ = 0;
  }

  // n1 * x + n2 * y + n3 * z + d/|n| = 0
  // plane_d_ = - plane_n_last_.dot(first_last_);

  plane_n_filt_ = lpf_alpha_ * plane_n_last_ + (1.0 - lpf_alpha_) * plane_n_filt_;

  // The acceleration offset represent how much `dde` should change
  // in the first_last_ point so this new point belong to a parallel
  // plane with `d` = 0, i.e., f_int = 0.
  dde_offset_ = plane_n_filt_.dot(first_last_) / plane_n_filt_(2);

  estimates_.data[0] = abs(plane_n_filt_(0));
  estimates_.data[1] = abs(plane_n_filt_(1));
  estimates_.data[2] = abs(plane_n_filt_(2));
  estimates_.data[3] = cluster_area_;
  estimates_.data[4] = dde_offset_;
}

}  // namespace impedance_identification

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::make_shared<impedance_identification::ImpedanceId>("identification");
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
