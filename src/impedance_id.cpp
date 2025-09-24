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

#include "impedance_reference_generator/impedance_id.hpp"

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
  phi_.setZero();
  error_.setZero();
  theta_.setZero();
  cov_k_.setIdentity();
  cov_k_ = 1'000 * cov_k_;
  // Initialize theta_
  theta_(0) = 10.0;     // 'k/m'
  theta_(1) = 6.32455;  // 'd/m'
  theta_(2) = 1.00;     // '1'

  double beta = 2 * M_PI * 0.25;  // cutoff = 1/4 * sampling frequency
  lpf_alpha_ = beta / (beta + 1);

  acceleration_filt_.setZero();
  last_state_.setZero();

  /* ISPI */
  zero_order_.setZero();
  first_order_.setZero();
  second_order_.setZero();
  cluster_.setZero();
  plane_normal_ << 1.0, 1.0, 1.0;
  plane_normal_last_ << 1.0, 1.0, 1.0;

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  axis_ = ::impedance_analysis::AxisMap[*(params_.axis.c_str())];

  input_subscriber_ = this->create_subscription<KinematicPose>(
    params_.controller_reference_topic, rclcpp::QoS(1).best_effort(),
    std::bind(&ImpedanceId::input_callback, this, std::placeholders::_1)
  );
  output_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    params_.controller_status_topic, rclcpp::QoS(1).best_effort(),
    std::bind(&ImpedanceId::output_callback, this, std::placeholders::_1)
  );

  // TODO(@me): use the SVD for long term estimation
  // timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(10), std::bind(&ImpedanceId::ispi_update, this)
  // );
  RCLCPP_INFO(get_logger(), "Running 'Z' identification on axis %s.", params_.axis.c_str());
  last_clock_ = get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceId::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // timer_.reset();
  input_subscriber_.reset();
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

void ImpedanceId::input_callback(const KinematicPose & msg)
{
  new_input_(0) = msg.pose.position.x;
  new_input_(1) = msg.pose_twist.linear.x;
  new_input_(2) = msg.pose_accel.linear.x;
}

void ImpedanceId::output_callback(const std_msgs::msg::Float64MultiArray & status_msg)
{
  delta_t_ = 1E-9 * static_cast<double>((get_clock()->now() - last_clock_).nanoseconds());

  new_output_(0) = status_msg.data[kDeviationIdx + axis_];  // deviation
  new_output_(1) = status_msg.data[kTwistDeviationIdx + axis_];  // deviation derivative

  step_detected_ = abs(status_msg.data[6 + axis_] - zero_order_(0)) > kPosDeltaThreshold;

  if (!step_detected_) {
    for (size_t i = kTimeWindow - 1; i > 0; --i) {
      first_order_(i) = first_order_(i - 1);
      zero_order_(i) = zero_order_(i - 1);
    }
    second_order_(0) = status_msg.data[kAccelDeviationIdx + axis_];;
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

  // rls_update();
  // or
  ispi_update();
  last_clock_ = get_clock()->now();
}

// TODO: HPF on these estimates
void ImpedanceId::rls_update()
{
  // rebuild the system state from x = e + x_d:
  state_ = new_output_ + new_input_.head<kSpaceDim * kOutputDim>();
  acceleration_ = (state_ - last_state_).tail<kSpaceDim>() / delta_t_;
  // Roll input/state story vectors
  last_state_ = state_;
  acceleration_filt_ = lpf_alpha_ * acceleration_ + (1.0 - lpf_alpha_) * acceleration_filt_;

  // Fill the regression vector
  phi_.head<kSpaceDim * kOutputDim>() = -new_output_;
  phi_(2) = new_input_(2);

  // Update Gain
  double den = lambda_ + phi_.transpose() * cov_k_ * phi_;
  gain_k_.noalias() = (cov_k_ * phi_) / den;
  // Update error
  error_.noalias() = acceleration_filt_ - theta_.transpose() * phi_;

  // New estimation
  theta_ = theta_ + gain_k_ * error_.transpose();
  // New covariance
  cov_k_ = (CovarianceMatrix::Identity() - gain_k_ * phi_.transpose()) * cov_k_ / lambda_;

  estimated_.data[0] = theta_(0);  // k/m
  estimated_.data[1] = theta_(1);  // d/m
  estimated_.data[2] = theta_(2);  // ~1
  estimated_.data[3] = state_(0);
  estimated_.data[4] = state_(1);
  // estimated_.data[5];
  estimated_.data[6] = acceleration_filt_(0);
  estimated_.data[7] = error_(0);
  param_publisher_->publish(estimated_);
}

// TODO: LPF on these estimates. Then do the complimentary filtering for RLS + ISPI
void ImpedanceId::ispi_update()
{
  // Fetch the contender point and compute the distance
  // to the last point on Cluster
  contender_point_ << zero_order_(2), first_order_(2), second_order_(0);
  contender_distance_ = (contender_point_ - cluster_.row(0)).norm();
  is_approved_ = contender_distance_ > kLengthLowerBound;  // pre-approved

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
    plane_normal_last_ = plane_normal_;

    theta_(0) = plane_normal_last_(0) / plane_normal_last_(2);  // k/m
    theta_(1) = plane_normal_last_(1) / plane_normal_last_(2);  // d/m
  }

  estimated_.data[0] = plane_normal_last_(0);
  estimated_.data[1] = plane_normal_last_(1);
  estimated_.data[2] = plane_normal_last_(2);
  estimated_.data[3] = theta_(0);
  estimated_.data[4] = theta_(1);
  estimated_.data[5] = cluster_area_;
  estimated_.data[6] = is_approved_;
  param_publisher_->publish(estimated_);
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
