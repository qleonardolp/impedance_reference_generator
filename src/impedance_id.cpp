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

  new_output_(0) = status_msg.data[kDeviationIdx + axis_];  // deviation
  new_output_(1) = status_msg.data[kTwistDeviationIdx + axis_];  // deviation derivative
  new_output_(2) = status_msg.data[kAccelDeviationIdx + axis_];  // deviation 2nd derivative

  step_detected_ =
    abs(status_msg.data[kDeviationIdx + axis_] - zero_order_(0)) > kPosDeltaThreshold;

  if (!step_detected_) {
    for (size_t i = kTimeWindow - 1; i > 0; --i) {
      first_order_(i) = first_order_(i - 1);
      zero_order_(i) = zero_order_(i - 1);
    }
    second_order_(0) = status_msg.data[kAccelDeviationIdx + axis_];
    first_order_(0) = status_msg.data[kTwistDeviationIdx + axis_];
    zero_order_(0) = status_msg.data[kDeviationIdx + axis_];

    // Compute `second_order_` from `first_order_` finite difference
    /*
    for (size_t i = 0; i < kPlaneWindow; i++) {
      second_order_(i) = 0;  // clear
      for (size_t k = 0; k < kFDCoeffcient.size(); k++) {
        // !! be careful here: (i + k) must be <= (kTimeWindow -1) !!
        second_order_(i) += kFDCoeffcient[k] * first_order_(i + k);
      }
      second_order_(i) /= period_;
    }
    */
  } else {
    second_order_.setZero();
    first_order_.setZero();
    zero_order_.setZero();
  }

  update_ispi();  // long-term trend
  update_rls();  // short-term trend
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
  // Fetch the contender point and compute the distance
  // to the last point on Cluster
  contender_point_ << zero_order_(0), first_order_(0), second_order_(0);
  contender_distance_ = (contender_point_ - cluster_.row(0)).norm();
  is_approved_ = contender_distance_ > kLengthlb;  // pre-approved

  if (is_approved_) {
    // Roll Cluster points (moving window)
    for (size_t k = kPlaneWindow - 1; k > 0; --k) {
      cluster_.row(k) = cluster_.row(k - 1);
    }
    cluster_.row(0) = contender_point_;

    // Centralize points
    cluster_centroid_ = cluster_.colwise().mean();
    cluster_centered_ = cluster_.rowwise() - cluster_centroid_;

    // Compute the SVD
    cluster_svd_.compute(cluster_centered_, Eigen::ComputeFullV);
    cluster_area_ =
      cluster_svd_.singularValues()(0) * cluster_svd_.singularValues()(1);
    least_sv_ = cluster_svd_.singularValues()(2);

    plane_normal_ = cluster_svd_.matrixV().rightCols<1>();
    // Fix sign flipping
    if (plane_normal_.dot(plane_normal_last_) < 0) {
      plane_normal_ = -plane_normal_;
    }

    // remember: n(2) = m / sqrt(k^2 + d^2 + m^2)
    if (plane_normal_(2) > kNormalddElb) {
      theta_svd_(0) = plane_normal_(0) / plane_normal_(2);  // k/m
      theta_svd_(1) = plane_normal_(1) / plane_normal_(2);  // d/m
      plane_normal_last_ = plane_normal_;
    }
  }

  estimated_.data[0] = plane_normal_last_(0);
  estimated_.data[1] = plane_normal_last_(1);
  estimated_.data[2] = plane_normal_last_(2);
  estimated_.data[3] = theta_svd_(0);
  estimated_.data[4] = theta_svd_(1);
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
