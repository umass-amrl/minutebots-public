// Copyright 2017 - 2019 srabiee@cs.umass.edu, slane@cs.umass.edu
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
#ifndef SRC_STATE_ESTIMATION_BALL_STATE_ESTIMATOR_H_
#define SRC_STATE_ESTIMATION_BALL_STATE_ESTIMATOR_H_

#include <utility>
#include <vector>

#include "constants/typedefs.h"
#include "datastructures/bounded_queue.h"
#include "logging/logger.h"
#include "state/direction.h"
#include "state/world_state.h"
#include "state_estimation/KF_const_vel.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "state_estimation/chip_kick_tester.h"

namespace estimation {
class BallStateEstimator {
 public:
  explicit BallStateEstimator(const direction::Direction& game_direction);
  ~BallStateEstimator();

  void Update(
      const SSLVisionProto::SSL_DetectionFrame& detection_frame,
      const state::PositionVelocityState& position_velocity_state,
      state::SharedState* shared_state,
      logger::Logger* logger);

  void Get2DState(Vector2f* position, Vector2f* velocity) const;
  void LogTrajectory(logger::Logger* logger);

  Eigen::Vector2f GetBallObservation() const;
  double GetBallObservationTime() const;
  unsigned int GetBallObservationCamera() const;

  // Returns true if a chip kick is being tracked with good confidence. Takes
  // some time steps after the chip kick onset for this to return true
  bool IsChipKickDetected() const;

  // Returns true as soon as a chip kick onset is detected. The state
  // and impact point might not yet be ready when it returns true.
  bool IsChipKickOnsetDetected() const;
  Eigen::Vector2f GetChipImpactPoint() const;
  Eigen::Vector3f GetChipInitialVel() const;
  double GetChipImpactTime() const;
  double GetChipOnsetTime() const;
  void UpdateCameraParams(const SSLVisionProto::SSL_GeometryData&
      geometry_packet);

  void SetBallState(
      const state::PositionVelocityState::BallPositionVelocity& ball);

 private:
  Eigen::Vector2d GetBestBallObservation(
      const SSLVisionProto::SSL_DetectionFrame& detection_frame,
      logger::Logger* logger) const;

  // state vector: [x, x_dot, y, y_dot, z, z_dot]'
  Vector6d ball_state_;
  estimation::KFConstVel KF_const_vel_;
  datastructures::BoundedQueue<Eigen::Vector2f> ball_estimate_history_;
  datastructures::BoundedQueue<Eigen::Vector2f> ball_detection_history_;

//   enum MotionMode { FLAT, CHIP_KICK };

//   MotionMode motion_mode_;
  double previous_observation_time_;
  uint previous_camera_id_;
  direction::Direction direction_;
  Eigen::Vector2d previous_observation_;

//   ChipKickTester chip_kick_evaluator_;

  const float kConfidenceThreshold_ = 0.5;
  static constexpr bool kLogBallData_ = false;
  const bool kUseChipKickDetection_ = true;
};
}  // namespace estimation
#endif  //  SRC_STATE_ESTIMATION_BALL_STATE_ESTIMATOR_H_
