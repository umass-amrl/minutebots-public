// Copyright 2018 - 2019 dbalaban@cs.umass.edu
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

#include "experimental_sim/sim_motion_model.h"


using std::default_random_engine;
using std::normal_distribution;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using logger::Logger;
using pose_2d::Pose2Df;

namespace experimental_simulator {

SimMotionModel::SimMotionModel() {
  generator.seed(random_seed);
  linVelActNoise =
      std::normal_distribution<float>(1.0, kLinearActivationNoise_);
  angVelActNoise =
      std::normal_distribution<float>(1.0, kAngularActivationNoise_);
  angVelDevNoise =
      std::normal_distribution<float>(0.0, kAngularDeviationNoise_);
}

void SimMotionModel::MoveRobot(const Pose2Df velocity_command,
                               const Pose2Df current_velocity,
                               const float time_slice,
                               Pose2Df* translation_pose,
                               Pose2Df* end_velocity) {
  Pose2Df realizedVelocity(velocity_command);

  if (kUseActivationNoise_) {
    realizedVelocity.translation = realizedVelocity.translation
                                   * linVelActNoise(generator);
    realizedVelocity.angle = realizedVelocity.angle
        * angVelActNoise(generator)
        + realizedVelocity.translation.norm() * angVelDevNoise(generator);
  }

  if (kUseActivationDrift_) {
    const Vector2f drift_direction(cos(-kActivationDriftAngle_),
                                   sin(-kActivationDriftAngle_));
    const Vector2f velocity = realizedVelocity.translation;
    const float drift_magnitude = kDriftAngleWeight_
                                  * velocity.dot(drift_direction);
    const Vector2f drift_vector = drift_magnitude * drift_direction;

    realizedVelocity.translation = velocity - drift_vector;
  }

  const float vel_mag = realizedVelocity.translation.norm();
  if (vel_mag < kMinimumVelocity_) {
    realizedVelocity.translation = Vector2f(0.0, 0.0);
  } else if (vel_mag > kSimMaximumVelocity_) {
      realizedVelocity.translation = kSimMaximumVelocity_
                                     * realizedVelocity.translation / vel_mag;
    }

  const Vector2f acceleration = (realizedVelocity.translation
                                 - current_velocity.translation) / time_slice;

  Vector2f translation;
  if (acceleration.norm() > kSimMaximumAcceleration_) {
    const Vector2f accel_revised =
      kSimMaximumAcceleration_ * acceleration / acceleration.norm();

    translation = current_velocity.translation * time_slice
                  + 0.5 * accel_revised * time_slice * time_slice;

    realizedVelocity.translation = current_velocity.translation
                                   + accel_revised * time_slice;
  } else {
    translation = time_slice * (realizedVelocity.translation
                                + current_velocity.translation) / 2.0;
  }

  const float angle_translation = time_slice * (realizedVelocity.angle
                                  + current_velocity.angle) / 2.0;

  (*end_velocity) = realizedVelocity;
  (*translation_pose) = Pose2Df(angle_translation, translation);
}


}  // namespace experimental_simulator
