// Copyright 2017-2019 srabiee@cs.umass.edu, slane@cs.umass.edu
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
#ifndef SRC_STATE_ESTIMATION_KF_CONST_VEL_H_
#define SRC_STATE_ESTIMATION_KF_CONST_VEL_H_


#include <utility>
#include <vector>
#include "constants/typedefs.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "logging/logger.h"

namespace estimation {
class KFConstVel{
 public:
  KFConstVel();
  ~KFConstVel() = default;

  static void ForwardPredict(const Eigen::Vector2f& position,
                             const Eigen::Vector2f& velocity,
                             double delta_t,
                             Eigen::Vector2f* predicted_positon,
                             Eigen::Vector2f* predicted_velocity,
                             logger::Logger* logger);

  static void ForwardPredict(const Vector6d& state,
                             double delta_t,
                             Vector6d* predicted_state,
                             logger::Logger* logger);

  static void Predict(const Vector6d& position,
                      const Matrix6d& covariance,
                      double delta_t,
                      Vector6d* output_position,
                      Matrix6d* output_covariance,
                      logger::Logger* logger);

  void Update(const Eigen::Vector2d& observation,
              double time,
              logger::Logger* logger);

  void Reset(const Eigen::Vector2d& observation,
             double delta_t);

  void SetState(const Vector6d& state);
  Vector6d GetCurrentState();
  Matrix6d GetCurrentCovariance();


  bool has_been_reset_ = false;

 private:
  static void CalculateProcessNoise(double delta_t, Matrix6d* process_noise);

  bool is_initialized_;
  // state vector: [x, x_dot, y, y_dot, z, z_dot]'
  Vector6d state_;

  // Covariance matrix
  Matrix6d covariance_;

  // Measurement noise covariance
  Eigen::Matrix2d measurement_noise_;

  // Observation matrix
  Eigen::Matrix<double, 2, 6> observation_matrix_;

  Eigen::Vector2d previous_observation_;
  double previous_observation_time_;

  constexpr static double kProcessNoiseWeight_ = 100.0;
  const double kResetThreshold_ = 100.0;
  const double kInitialCovarianceWeight_ = 10.0f;
};
}  // namespace estimation

#endif  // SRC_STATE_ESTIMATION_KF_CONST_VEL_H_
