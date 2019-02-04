// Copyright 2017-2018 srabiee@cs.umass.edu, slane@cs.umass.edu
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

#include "state_estimation/ball_state_estimator.h"

#include <glog/logging.h>

#include <limits>

#include "constants/constants.h"
#include "datastructures/bounded_queue.h"
#include "logging/logger.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "state/team.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "util/timer.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

STANDARD_USINGS;
using state::WorldRobot;
using state::WorldState;
using team::Team;
using geometry::EuclideanDistance;
using Eigen::Vector3f;
using Eigen::Vector3d;
using Eigen::Vector2f;
using Eigen::Vector2d;
using logger::Logger;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionBall;
using datastructures::BoundedQueue;
using direction::Direction;
using logger::Logger;
using std::numeric_limits;

namespace estimation {
BallStateEstimator::BallStateEstimator(
    const direction::Direction& game_direction)
    :  // motion_mode_(FLAT),
      ball_estimate_history_(kFrameQueueSize),
      ball_detection_history_(kFrameQueueSize),
      previous_observation_time_(0.0),
      previous_camera_id_(0),
      direction_(game_direction),
      previous_observation_(0, 0) {
  ball_state_ = Eigen::VectorXd::Zero(6, 1);
  ball_state_(2) = kBallZeroHeight;
//   chip_kick_evaluator_.direction_ = game_direction;
//   chip_kick_evaluator_.test_mode = 3;  // partial optimization
//   chip_kick_evaluator_.test_mode = 4;  // full optimization
//   chip_kick_evaluator_.test_mode = 5;  // Cmp partial and full optimization
//   chip_kick_evaluator_.test_mode = 2;  // Manual ball throwing
//   chip_kick_evaluator_.Reset();
}

BallStateEstimator::~BallStateEstimator() {}

void BallStateEstimator::LogTrajectory(logger::Logger* logger) {
  for (size_t k = 1; k < ball_detection_history_.Size(); k++) {
    logger->AddLine(ball_detection_history_[k],
                    ball_detection_history_[k - 1],
                    1,
                    0,
                    0,
                    1);
    logger->AddLine(ball_estimate_history_[k],
                    ball_estimate_history_[k - 1],
                    0,
                    0,
                    1,
                    1);
  }
}

void BallStateEstimator::Update(
    const SSL_DetectionFrame& detection,
    const state::PositionVelocityState& position_velocity_state,
    state::SharedState* shared_state,
    Logger* logger) {
  uint camera_id = detection.camera_id();
  double timestamp = detection.t_capture();


  if (timestamp <= previous_observation_time_) {
    logger->LogPrint("Recieved ball observations out of order");
    return;
  }

  Vector2d ball_raw_pos;

  // Exploit all captured frames for chip kick detection (do not prune
  // consequtive frames from different cameras)
  if (!detection.balls().empty()) {
    // Go through the input local ssl_vision queue of detection frames with
    // ball detections and take the most confident detection from each frame.
    ball_raw_pos  = GetBestBallObservation(detection, logger);

    // Run chip kick detector
    if (kUseChipKickDetection_) {
      state::PositionVelocityState updated_position_velocity_state;
      updated_position_velocity_state = position_velocity_state;
      Vector2f ball_raw_pos_float(static_cast<float>(ball_raw_pos.x()),
                                  static_cast<float>(ball_raw_pos.y()) );

      *(updated_position_velocity_state.GetMutableBallPositionVelocity()) =
        state::PositionVelocityState::BallPositionVelocity(
            Vector2f(0, 0), Vector2f(0, 0),
            ball_raw_pos_float,
            timestamp,
            camera_id);

//       chip_kick_evaluator_.Update(updated_position_velocity_state,
//                                   shared_state,
//                                   logger);
    }
  }

  // Prunes out detection frames from a different camera
  if (!detection.balls().empty() && (camera_id == previous_camera_id_ ||
      timestamp - previous_observation_time_ > 2.0 / kTransmitFrequency)) {
    previous_camera_id_ = camera_id;
    previous_observation_time_ = timestamp;


    logger->LogPrint("Ball Observation: %f, %f",
                     ball_raw_pos.x(),
                     ball_raw_pos.y());


    KF_const_vel_.Update(ball_raw_pos, timestamp, logger);
    ball_state_ = KF_const_vel_.GetCurrentState();

    ball_estimate_history_.Add(ball_state_.head(2).cast<float>());
    ball_detection_history_.Add(ball_raw_pos.cast<float>());
    previous_observation_ = ball_raw_pos;
  } else {
    Vector6d next_state(ball_state_);
    double delta_t = timestamp - previous_observation_time_;
    KF_const_vel_.ForwardPredict(ball_state_,
                                 delta_t,
                                 &next_state,
                                 logger);
  }
}

void BallStateEstimator::Get2DState(Vector2f* position,
                                    Vector2f* velocity) const {
  *position = ball_state_.head(2).cast<float>();
  *velocity = ball_state_.segment(3, 2).cast<float>();
}


Vector2f BallStateEstimator::GetBallObservation() const {
  return previous_observation_.cast<float>();
}

double BallStateEstimator::GetBallObservationTime() const {
  return previous_observation_time_;
}

unsigned int BallStateEstimator::GetBallObservationCamera() const {
  return previous_camera_id_;
}

// Vector6d BallStateEstimator::Predict(double time) {
//   Vector6d predicted_state = Vector6d::Zero();
//   if (motion_mode_ == FLAT) {
//     // TODO(srabiee): Reset the KF if the motion mode has switched back to
       // FLAT
//     if (KF_const_vel_.has_been_reset_) {
//       KF_const_vel_.Predict(time);
//       predicted_state = KF_const_vel_.GetPredictedState();
//     }
//   } else if (motion_mode_ == CHIP_KICK) {
//     // TODO(srabiee): Add the chip kick estimator
//   }
//
//   return predicted_state;
// }

Vector2d BallStateEstimator::GetBestBallObservation(
    const SSLVisionProto::SSL_DetectionFrame& detection_frame,
    Logger* logger) const {
  static const bool kDebug = true;
  if (kDebug) {
    logger->LogPrintPush("GetBestBallObservation");
    logger->LogPrint("Previous Observation: %f, %f",
                         previous_observation_.x(),
                         previous_observation_.y());
  }
  Vector2d best_position = Vector2d::Zero();
  double min_distance = numeric_limits<float>::max();

  for (const SSL_DetectionBall& ball : detection_frame.balls()) {
    float confidence = ball.confidence();
    Vector2d ball_raw_pos = Vector2d(static_cast<double>(ball.x()),
                                     static_cast<double>(ball.y()));

    // Reject the detection if it has low confidence
    if (confidence > kConfidenceThreshold_) {
      if (direction_ != Direction::POSITIVE) {
        ball_raw_pos = -1 * ball_raw_pos;
      }

      double distance = EuclideanDistance(ball_raw_pos,
                                          previous_observation_);
      if (kDebug) {
        logger->LogPrint("Observation: %f, %f",
                         ball_raw_pos.x(),
                         ball_raw_pos.y());
        logger->LogPrint("Confidence: %f", confidence);
        logger->LogPrint("Distance: %f", distance);
      }

      if (distance < min_distance) {
        min_distance = distance;
        best_position = ball_raw_pos;
      }
    }
  }
  if (kDebug) {
    logger->Pop();
  }

  if (kLogBallData_) {
    static ScopedFile predictions_fid("ball_data.csv", "w");
    fprintf(predictions_fid,
            "%f, %f, %f\n",
            detection_frame.t_sent(),
            best_position.x(),
            best_position.y());
  }

  return best_position;
}

bool BallStateEstimator::IsChipKickDetected() const {
  //  chip_kick_evaluator_.IsChipKickDetected();
  return false;
}

bool BallStateEstimator::IsChipKickOnsetDetected() const {
  //  chip_kick_evaluator_.IsChipKickOnsetDetected();
  return false;
}


Vector2f BallStateEstimator::GetChipImpactPoint() const {
  //  chip_kick_evaluator_.GetImpactPoint();
  return {0, 0};
}

double BallStateEstimator::GetChipImpactTime() const {
  //  chip_kick_evaluator_.GetImpactTime();
  return 0;
}

double BallStateEstimator::GetChipOnsetTime() const {
  //  chip_kick_evaluator_.GetChipShotTime();
  return 0;
}


Vector3f BallStateEstimator::GetChipInitialVel() const {
  //  chip_kick_evaluator_.GetInitialVel();
  return {0, 0, 0};
}

void
BallStateEstimator::UpdateCameraParams(const SSLVisionProto::SSL_GeometryData&
    geometry_packet) {
  // chip_kick_evaluator_.UpdateCameraParams(geometry_packet);
}
}  // namespace estimation
