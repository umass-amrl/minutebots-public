// Copyright 2017 - 2018 slane@cs.umass.edu
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

#include "tactics/pd_controller.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry::SafeVectorNorm;
using math_util::AngleMod;
using math_util::Sign;
using state::SharedRobotState;
using state::WorldState;
using std::atan2;
using std::cos;
using std::endl;
using std::map;
using std::max;
using std::min;
using std::sin;
using std::abs;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;

namespace tactics {

PDController::PDController(const WorldState& world_state,
                           TacticArray* tactic_list, SharedState* shared_state,
                           OurRobotIndex our_robot_index,
                           state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      translation_complete_(false),
      rotation_complete_(false) {}

PDController::~PDController() {}

void PDController::Run() {
  static const bool kDebug = false;
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  logger->LogPrintPush("PD Controller");

  static constexpr float kAngularProportional = 5;  // 12 / 2.0;
  // const float kAngularDerivative = -0.8;
  static constexpr float kAngularDerivative = -0.01;  // -0.35 / 2.0;

  // const float kTranslationProportional = 15.0;
  // const float kTranslationDerivative = -3.0;
  static constexpr float kTranslationProportional = 9.5 / 0.8;  // 250.0;
  static constexpr float kTranslationDerivative = -0.1 / 1.1;   // -30.0;

  // Deadzone for commands around the center in mm.
  static constexpr float kTranslationalDeadZone = 7;
  //   logger::Logger* robot_logger =
  //     soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  const Pose2Df& current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  current_velocity.translation =
      Rotation2Df(current_pose.angle) * current_velocity.translation;

  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;

  const Vector2f translation_error =
      (goal_.translation - current_pose.translation);

  Pose2Df target_velocity(0, 0, 0);
  if (translation_complete_ ||
      translation_error.squaredNorm() < Sq(kTranslationalDeadZone)) {
    state->should_stop_linear = true;
    target_velocity.translation.setZero();
  } else {
    const Vector2f translational_proportional_term =
        kTranslationProportional * translation_error;
    logger->LogPrint("Trans P: %f, %f", translational_proportional_term.x(),
                     translational_proportional_term.y());

    const Vector2f translational_derivative_term =
        kTranslationDerivative * current_velocity.translation;
    logger->LogPrint("Trans D: %f, %f", translational_derivative_term.x(),
                     translational_derivative_term.y());

    target_velocity.translation =
        translational_proportional_term + translational_derivative_term;
  }

  const float angular_error = AngleMod(goal_.angle - current_pose.angle);

  if (rotation_complete_) {
    state->should_stop_angular = true;
    target_velocity.angle = 0;
  } else {
    logger->LogPrint("Angular error: %f", angular_error);

    // Calculate desired rotational velocity
    const float angular_proportional_term =
        kAngularProportional * angular_error;

    // float angular_derivative = 0.0f;
    const float angular_derivative_term =
        kAngularDerivative * current_velocity.angle;

    logger->LogPrint("Angular P: %f", angular_proportional_term);

    logger->LogPrint("Angular D: %f", angular_derivative_term);

    target_velocity.angle =
        min(static_cast<float>(kMaxRobotRotVel),
            angular_proportional_term + angular_derivative_term);
  }

  float max_velocity = kMaxRobotVelocity;
  if (!soccer_state_->IsNormalPlay()) {
    max_velocity = std::min(1250.0f, kMaxRobotVelocity);
  }

  float linear_speed = SafeVectorNorm(target_velocity.translation);
  if (fabs(linear_speed) > max_velocity) {
    target_velocity.translation =
        max_velocity * target_velocity.translation / linear_speed;
  }

  Pose2Df target_acceleration;
  target_acceleration.translation =
      (target_velocity.translation - current_velocity.translation) /
      kTransmitPeriodSeconds;
  float linear_acceleration = SafeVectorNorm(target_acceleration.translation);
  if (fabs(linear_acceleration) > kMaxRobotAcceleration) {
    target_acceleration.translation = kMaxRobotAcceleration *
                                      target_acceleration.translation /
                                      linear_acceleration;
  }

  target_acceleration.angle =
      (target_velocity.angle - current_velocity.angle) / kTransmitPeriodSeconds;
  if (fabs(target_acceleration.angle) > kMaxRobotRotAccel) {
    target_acceleration.angle =
        Sign(target_acceleration.angle) * kMaxRobotRotAccel;
  }

  if (kDebug) {
    logger->LogPrint("Current Velocity: %f, %f, %f",
                     current_velocity.translation.x(),
                     current_velocity.translation.y(), current_velocity.angle);

    logger->LogPrint("Target Velocity: %f, %f, %f",
                     target_velocity.translation.x(),
                     target_velocity.translation.y(), target_velocity.angle);

    logger->LogPrint(
        "Acceleration: %f, %f, %f", target_acceleration.translation.x(),
        target_acceleration.translation.y(), target_acceleration.angle);
  }

  state->acceleration_command = target_acceleration;

  rotation_complete_ = false;
  translation_complete_ = false;

  logger->Pop();
}

void PDController::Reset() {
  rotation_complete_ = false;
  translation_complete_ = false;
}

void PDController::Init() {}

void PDController::SetGoal(const pose_2d::Pose2Df& pose) { goal_ = pose; }

void PDController::SetRotationComplete() { rotation_complete_ = true; }

void PDController::SetTranslationComplete() { translation_complete_ = true; }
}  // namespace tactics
