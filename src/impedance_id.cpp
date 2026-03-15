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

  estimated_.data.resize(8, 0.0);
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

  double beta = 2 * M_PI * 0.25;  // cutoff = 1/4 * fs
  lpf_alpha_ = beta / (beta + 1);

  /* RLS */
  phi_.setZero();
  error_.setZero();
  theta_.setZero();
  cov_k_.setIdentity();
  cov_k_ *= 100.0;
  rls_gain_den_ = 1.0;

  // Initialize theta_
  theta_(0) = params_.expected_stiffness / params_.expected_mass;  // 'k/m'
  theta_(1) = params_.expected_damping / params_.expected_mass;  // 'd/m'
  theta_last_ = theta_;
  theta_fused_ = theta_;

  /* ISPI */
  last_point_.setZero();
  zero_order_.setZero();
  first_order_.setZero();
  second_order_.setZero();
  theta_svd_.setZero();
  cluster_.setZero();
  plane_normal_ <<
    Eigen::Vector3d(
      params_.expected_stiffness,
      params_.expected_damping,
      params_.expected_mass).normalized();

  plane_normal_last_ = plane_normal_;
  three_points_ = 0;
  downsample_ = 0;

  output_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    params_.controller_status_topic, rclcpp::QoS(1).best_effort(),
    std::bind(&ImpedanceId::output_callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(get_logger(), "Running 'Z' identification on axis %s.", params_.axis.c_str());
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

  new_output_(0) = status_msg.data[kDeviationIdx + axis_];       // deviation
  new_output_(1) = status_msg.data[kTwistDeviationIdx + axis_];  // deviation derivative
  new_output_(2) = status_msg.data[kAccelDeviationIdx + axis_];  // deviation 2nd derivative

  // downsample_++;
  // if (0 == downsample_ % 4) {
  step_detected_ =
    abs(status_msg.data[kDeviationIdx + axis_] - zero_order_(0)) > kPosDeltaThreshold;
  estimated_.data[7] = step_detected_;

  if (!step_detected_) {
    new_point_(0) = status_msg.data[kDeviationIdx + axis_];
    new_point_(1) = status_msg.data[kTwistDeviationIdx + axis_];
    new_point_(2) = status_msg.data[kAccelDeviationIdx + axis_];
  }

  if ((new_point_ - last_point_).norm() > kLengthlb) {
    for (size_t i = kPlaneWindow - 1; i > 0; --i) {
      second_order_(i) = second_order_(i - 1);
      first_order_(i) = first_order_(i - 1);
      zero_order_(i) = zero_order_(i - 1);
    }
    second_order_(0) = new_point_(2);
    first_order_(0) = new_point_(1);
    zero_order_(0) = new_point_(0);
    three_points_++;
  }

  if (0 == three_points_ % 3) {
    // Compute only once three new points are fetched.
    // This is to decouple the effect of d (`f_int`) between
    // planes with different d.
    update_ispi();
    three_points_ = 0;
  }
  // downsample_ = 0;
  // }

  // update_rls();
  param_publisher_->publish(estimated_);
  last_clock_ = get_clock()->now();
}

void ImpedanceId::update_rls()
{
  // Update the regression vector
  phi_ = -new_output_.head<kPhiSize>();

  // Update Gain
  rls_gain_den_ = lambda_ + phi_.transpose() * cov_k_ * phi_;
  gain_k_.noalias() = (cov_k_ * phi_) / rls_gain_den_;

  // Update error (is blowing up...)
  error_.noalias() = new_output_.tail<kSpaceDim>() - theta_.transpose() * phi_;

  // New estimation
  theta_ = theta_ + gain_k_ * error_.transpose();
  // New covariance
  cov_k_ = (CovarianceMatrix::Identity() - gain_k_ * phi_.transpose()) * cov_k_ / lambda_;

  // TODO(@qleonardolp): review this fusion formulation
  theta_fused_ = (1.0 - lpf_alpha_) * theta_svd_ +
    lpf_alpha_ * (theta_fused_ + theta_ - theta_last_);
  theta_last_ = theta_;

  estimated_.data[5] = theta_fused_(0);  // k/m
  estimated_.data[6] = theta_fused_(1);  // d/m
  estimated_.data[7] = error_(0);
}

void ImpedanceId::update_ispi()
{
  // (y_1 - y_3) * z_2
  ispi_est_(0) = (first_order_(0) - first_order_(2)) * second_order_(1);
  // z_1 * x_3 - z_3 * x_1
  ispi_est_(1) = second_order_(0) * zero_order_(2) - second_order_(2) * zero_order_(0);
  // (y_3 - y_1) * x_2
  ispi_est_(2) = (first_order_(2) - first_order_(0)) * zero_order_(1);
  // x_3 * y_2 * z_1 - z_3 * y_2 * x_1
  ispi_est_(3) =
    zero_order_(2) * first_order_(1) * second_order_(0) -
    second_order_(2) * first_order_(1) * zero_order_(0);

  v1_ << zero_order_(1) - zero_order_(0), first_order_(1) - first_order_(0), second_order_(1) - second_order_(0);
  v2_ << zero_order_(2) - zero_order_(0), first_order_(2) - first_order_(0), second_order_(2) - second_order_(0);

  estimated_.data[0] = ispi_est_(0);
  estimated_.data[1] = ispi_est_(1);
  estimated_.data[2] = ispi_est_(2);
  estimated_.data[3] = ispi_est_(3);
  estimated_.data[4] = v1_.norm();
  estimated_.data[5] = v2_.norm();
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
