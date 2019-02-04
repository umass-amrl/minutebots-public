// Copyright 2017 - 2018 srabiee@cs.umass.edu, slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include "state_estimation/KF_const_accel.h"
#include <glog/logging.h>

#include "eigen3/Eigen/Core"
#include "logging/logger.h"
#include "math/geometry.h"

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::EuclideanDistance;
using math_util::Pow;
using logger::Logger;
using std::endl;
using std::abs;

namespace estimation {

constexpr double KFConstAccel::kProcessNoiseWeight_;
const double KFConstAccel::kBallAcceleration_ =
    -static_cast<double>(kBallAcceleration);


KFConstAccel::KFConstAccel()
    : is_initialized_(false),
      previous_observation_(0, 0),
      previous_observation_time_(0) {
  measurement_noise_ = Sq(1.5)*Matrix2d::Identity();

  observation_matrix_ << 1, 0, 0, 0, 0, 0,
                         0, 1, 0, 0, 0, 0;

  state_.setZero();
  state_(2) = kBallZeroHeight;
  covariance_.setZero();
}

void KFConstAccel::ForwardPredict(const Vector2f& position,
                                 const Vector2f& velocity,
                                 double delta_t,
                                 Vector2f* predicted_position,
                                 Vector2f* predicted_velocity,
                                 Logger* logger) {
  Vector6d state = Vector6d::Zero();
  state.head(2) = position.cast<double>();
  state(2) = kBallZeroHeight;
  state.segment(3, 2) = velocity.cast<double>();

  Matrix6d covariance = Matrix6d::Identity();

  Vector6d output_state = Vector6d::Zero();
  Matrix6d output_covariance = Matrix6d::Identity();

  Predict(state,
          covariance,
          delta_t,
          &output_state,
          &output_covariance,
          logger);

  *predicted_position = output_state.head(2).cast<float>();
  *predicted_velocity = output_state.segment(3, 2).cast<float>();
}

void KFConstAccel::ForwardPredict(const Vector6d& state,
                                 double delta_t,
                                 Vector6d* predicted_state,
                                 Logger* logger) {
  Matrix6d covariance = Matrix6d::Identity();
  Matrix6d output_covariance = Matrix6d::Identity();
  Predict(state,
          covariance,
          delta_t,
          predicted_state,
          &output_covariance,
          logger);
}


void KFConstAccel::Predict(const Vector6d& state,
                          const Matrix6d& covariance,
                          double delta_t,
                          Vector6d* output_state,
                          Matrix6d* output_covariance,
                          Logger* logger) {
  // Check if detections are received out of sequence
  if (delta_t < 0.0) {
    logger->LogPrint("Out of sequence observation was found. Delta T = %f",
                     delta_t);
    return;
  }

  // State transition matrix
  Matrix6d jacobian = Matrix6d::Identity();

  const double& x = state(0);
  const double& y = state(1);
  const double& vx = state(3);
  const double& vy = state(4);

  double half_dt_squared = Sq(delta_t)/2.0;

  double speed = sqrt(Sq(vx) + Sq(vy));
  double a_x = vx/speed * kBallAcceleration_;
  double a_y = vy/speed * kBallAcceleration_;
  double speed_cubed = Pow(speed, 3);
  *output_state = state;

  if (speed > abs(kBallAcceleration_ * delta_t)) {
    // Ball is actually moving
    (*output_state)(0) = x + vx * delta_t + a_x * half_dt_squared;
    (*output_state)(1) = y + vy * delta_t + a_y * half_dt_squared;
    (*output_state)(3) = vx + a_x * delta_t;
    (*output_state)(4) = vy + a_y * delta_t;

    jacobian(0, 3) =
        delta_t + half_dt_squared * kBallAcceleration_ * Sq(vy)/speed_cubed;
    jacobian(0, 4) = -half_dt_squared * kBallAcceleration_ * vx * vy /
                      speed_cubed;

    jacobian(1, 3) = -half_dt_squared * kBallAcceleration_ * vx * vy /
                     speed_cubed;
    jacobian(1, 4) =
        delta_t + half_dt_squared * kBallAcceleration_ * Sq(vx)/speed_cubed;

    jacobian(3, 3) = half_dt_squared * kBallAcceleration_ * Sq(vy)/speed_cubed;
    jacobian(3, 4) = -half_dt_squared * kBallAcceleration_ * vx * vy /
                     speed_cubed;
    jacobian(4, 3) = -half_dt_squared * kBallAcceleration_ * vx * vy /
                     speed_cubed;
    jacobian(4, 4) = half_dt_squared * kBallAcceleration_ * Sq(vx)/speed_cubed;
  } else if (speed > abs(kBallAcceleration_ * kTransmitPeriodSeconds)) {
    // TODO(slane): Fix Jacobian in this case
    double stopping_distance = Sq(speed) / (2.0 * abs(kBallAcceleration_));
    (*output_state)(0) = x + vx/speed * stopping_distance;
    (*output_state)(1) = y + vy/speed * stopping_distance;
    (*output_state)(3) = 0;
    (*output_state)(4) = 0;
  } else {
    (*output_state)(3) = 0;
    (*output_state)(4) = 0;
  }
  // Process noise covariance matrix
  Matrix6d process_noise;
  CalculateProcessNoise(delta_t, &process_noise);
  *output_covariance = jacobian * covariance * jacobian.transpose() +
                       process_noise;

  return;
}

void KFConstAccel::Reset(const Vector2d& observation,
                        double delta_t) {
  state_.head(2) = observation;
  state_(2) = kBallZeroHeight;
  state_.segment(3, 2) = (observation - previous_observation_)/delta_t;
  state_(5) = 0.0;

  Matrix6d Q;
  CalculateProcessNoise(delta_t, &Q);

  covariance_ = kInitialCovarianceWeight_ * Q;
}

void KFConstAccel::Update(const Vector2d& observation,
                         double time,
                         Logger* logger) {
  if (!is_initialized_) {
    previous_observation_ = observation;
    previous_observation_time_ = time;

    // Reseting with previous_observation_ = current observation sets the state
    // To the observation
    Reset(observation, 1.0);
    is_initialized_ = true;
  }

  // First, performs the prediction step given the current time
  double delta_t = time - previous_observation_time_;
  if (delta_t < 0.0) {
    LOG(INFO) << "Out of sequence observation was found. Delta_t: "
              << time - previous_observation_time_ << endl;
    return;
  }

  Vector6d predicted_state(state_);
  Matrix6d predicted_covariance(covariance_);
  Predict(state_,
          covariance_,
          delta_t,
          &predicted_state,
          &predicted_covariance,
          logger);
  double position_error = EuclideanDistance(observation,
                                            Vector2d(predicted_state.head(2)));

//   double velocity_error = EuclideanDistance(
//       Vector2d(observation - previous_observation_),
//       Vector2d(predicted_state.segment(3, 2)));


  if (position_error > kResetThreshold_) {
    logger->LogPrint("Resetting");
    Reset(observation, delta_t);
    previous_observation_time_ = time;
    previous_observation_ = observation;
    return;
  }

  state_ = predicted_state;
  covariance_ = predicted_covariance;

  // Caclulate the Kalman gain
  Matrix<double, 6, 2> K = (covariance_ * observation_matrix_.transpose()) *
                           (observation_matrix_ * covariance_ *
                            observation_matrix_.transpose() +
                            measurement_noise_).inverse();

  state_ = state_ + K * (observation - (observation_matrix_ * state_));
  Matrix6d identity_matrix = Matrix6d::Identity();
  covariance_ = (identity_matrix - K * observation_matrix_) * covariance_;

  previous_observation_time_ = time;
  previous_observation_ = observation;
  return;
}

Vector6d KFConstAccel::GetCurrentState() {
  return state_;
}

Matrix6d KFConstAccel::GetCurrentCovariance() {
  return covariance_;
}

void KFConstAccel::CalculateProcessNoise(double delta_t,
                                        Matrix6d* process_noise) {
    *process_noise <<
      0,  0, 0,   0, 0,  0,
      0,  0, 0,   0, 0,  0,
      0,  0, 0,   0, 0,  0,
      0,  0, 0,  1000.0, 0, 0,
      0,  0, 0,  0.0,   1000.0, 0,
      0,  0, 0,  0.0,   0, 0;
}


}  // namespace estimation

