// Copyright 2017-2018 slane@cs.umass.edu
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

#include "tactics/forward_backward.h"

#include <cmath>

#include <map>
#include <memory>
#include <string>

#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "tactics/ntoc_controller.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using state::WorldState;
using state::SharedState;
using std::map;
using std::unique_ptr;
using tactics::NTOC_Controller;
using tactics::TacticIndex;
using math_util::Sq;

namespace tactics {
ForwardBackward::ForwardBackward(const WorldState& world_state,
                                 TacticArray* tactic_list,
                                 SharedState* shared_state,
                                 OurRobotIndex our_robot_index,
                                 state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      pose1_(0, 0, 0),
      pose2_(0, 0, 0),
      goal_(0, 0, 0),
      wait_start_time_(0),
      last_moving_state_(GET_TO_INIT_POSE),
      goal_angle_(0),
      our_robot_index_(our_robot_index) {}

std::string BoolToString(const bool b) { return (b ? "true" : "false"); }

bool ForwardBackward::IsNearGoal(logger::Logger* logger) const {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_vel =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const bool translationally_near =
      (goal_.translation - current_pose.translation).squaredNorm() <
      Sq(kDistanceThreshold + 5);
  const bool angle_near =
      (fabs(AngleMod(goal_.angle - current_pose.angle)) < kAngleThreshold * 5);
  const bool translational_vel_near =
      current_vel.translation.squaredNorm() < Sq(kLinearVelocityThreshold);
  const bool angular_vel_near =
      fabs(current_vel.angle) < kAngularVelocitythreshold;

  logger->LogPrint(
      "translationally_near: %s angle_near: "
      "%s translational_vel_near: %s angular_vel_near %s",
      BoolToString(translationally_near), BoolToString(angle_near),
      BoolToString(translational_vel_near), BoolToString(angular_vel_near));
  return translationally_near && angle_near && translational_vel_near &&
         angular_vel_near;
}

bool ForwardBackward::IsInWaitPeriod() const {
  return false;
  return (GetMonotonicTime() - wait_start_time_) < kWaitTime;
}

void ForwardBackward::Run() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_vel =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  if (IsNearGoal(logger) && !IsInWaitPeriod()) {
    switch (execution_state_) {
      case GET_TO_INIT_POSE:
        execution_state_ = TURNING;
        last_moving_state_ = BACKWARD;
        break;
      case TURNING:
        execution_state_ = WAITING;
        wait_start_time_ = GetMonotonicTime();
        break;
      case FORWARD:
        execution_state_ = WAITING;
        last_moving_state_ = FORWARD;
        wait_start_time_ = GetMonotonicTime();
        break;
      case BACKWARD:
        execution_state_ = WAITING;
        last_moving_state_ = BACKWARD;
        wait_start_time_ = GetMonotonicTime();
        break;
      case WAITING:
        if (last_moving_state_ == FORWARD) {
          execution_state_ = BACKWARD;
        } else {
          execution_state_ = FORWARD;
        }
    }
  } else {
    logger->LogPrint("IsNearGoal(): %s IsInWaitPeriod(): %s",
                     (IsNearGoal(logger) ? "true" : "false"),
                     (IsInWaitPeriod() ? "true" : "false"));
  }

  if (execution_state_ == GET_TO_INIT_POSE &&
      (goal_.translation - current_pose.translation).squaredNorm() <
          Sq(kDistanceThreshold) &&
      current_vel.translation.squaredNorm() < Sq(kLinearVelocityThreshold)) {
    execution_state_ = TURNING;
  }

  NTOC_Controller* controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  controller->TurnOnAngleRelaxation();

  switch (execution_state_) {
    case GET_TO_INIT_POSE:
      goal_ = pose1_;
      goal_.angle = goal_angle_;
      logger->LogPrint("In GET_TO_INIT_POSE");
      break;
    case TURNING:
      goal_ = current_pose;
      goal_.angle = goal_angle_;
      controller->SetTranslationComplete();
      logger->LogPrint("In TURNING");
      break;
    case FORWARD:
      goal_ = pose2_;
      logger->LogPrint("In FORWARD");
      break;
    case BACKWARD:
      goal_ = pose1_;
      logger->LogPrint("In BACKWARD");
      break;
    case WAITING:
      logger->LogPrint("In WAITING");
      break;
  }

  Tactic* navigator = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  navigator->SetGoal(goal_);
  navigator->Run();
}

void ForwardBackward::Reset() {
  execution_state_ = GET_TO_INIT_POSE;
  Vector2f pos1(4000, 1300 - static_cast<float>(our_robot_index_) * 200);
  Vector2f pos2(-4000, 1300 - static_cast<float>(our_robot_index_) * 200);
  goal_angle_ = 0;
  pose1_.Set(0.0, pos1);
  pose2_.Set(0.0, pos2);
  goal_ = pose1_;
}

void ForwardBackward::Init() { Reset(); }

void ForwardBackward::SetGoal(const Pose2Df& pose) {}
}  // namespace tactics
