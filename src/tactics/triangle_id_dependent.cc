// Copyright 2017 - 2018 kvedder@umass.edu
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

#include "tactics/triangle_id_dependent.h"

#include <cmath>

#include <map>
#include <memory>

#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_state.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using state::WorldState;
using state::SharedState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;

namespace tactics {
TriangleIdDependent::TriangleIdDependent(
    const WorldState& world_state, TacticArray* tactic_list,
    SharedState* shared_state, OurRobotIndex our_robot_index,
    state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

void TriangleIdDependent::Run() {
  const int& ssl_vision_id =
      world_state_.GetOurRobots().Get(our_robot_index_).ssl_vision_id;
  poseA.Set(0.0f, Vector2f(1000 - 300 * ssl_vision_id, 1000));
  poseB.Set(0.0f, Vector2f(2500 - 300 * ssl_vision_id, 2000));
  poseC.Set(0.0f, Vector2f(3000 - 300 * ssl_vision_id, -2000));
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  robot_logger->LogPrint("Triangle ID Dependent: %d", ssl_vision_id);
  robot_logger->Push();

  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  if (execution_state == TURN &&
      fabs(current_pose.angle - goal.angle) < kNavigationAngularThreshold &&
      fabs(current_velocity.angle) < kNavigationLinearVelocityThreshold) {
    execution_state = FORWARD;
  } else if (execution_state == WAIT &&
             world_state_.world_time_ >= wait_start_time_ + kWaitDuration_) {
    execution_state = TURN;
  } else if (execution_state == FORWARD &&
             (current_pose.translation - goal.translation).norm() <
                 kDistanceThreshold &&
             current_velocity.translation.norm() < kLinearVelocityThreshold) {
    switch (goal_pose) {
      case POSEA:
        goal_pose = POSEB;
        break;
      case POSEB:
        goal_pose = POSEC;
        break;
      case POSEC:
        goal_pose = POSEA;
    }
    execution_state = WAIT;
    wait_start_time_ = world_state_.world_time_;
  }

  Vector2f displacement(0, 0);
  switch (execution_state) {
    case WAIT:
      robot_logger->LogPrint("Waiting");
      goal = current_pose;
      break;
    case TURN:
      robot_logger->LogPrint("Turning");
      goal = current_pose;
      switch (goal_pose) {
        case POSEA:
          displacement = poseA.translation - current_pose.translation;
          break;
        case POSEB:
          displacement = poseB.translation - current_pose.translation;
          break;
        case POSEC:
          displacement = poseC.translation - current_pose.translation;
          break;
      }
      break;
    case FORWARD:
      robot_logger->LogPrint("Going Forward");
      switch (goal_pose) {
        case POSEA:
          robot_logger->LogPrint("Going Forward to %f, %f",
                                 poseA.translation.x(), poseA.translation.y());
          goal = poseA;
          break;
        case POSEB:
          robot_logger->LogPrint("Going Forward to %f, %f",
                                 poseB.translation.x(), poseB.translation.y());
          goal = poseB;
          break;
        case POSEC:
          robot_logger->LogPrint("Going Forward to %f, %f",
                                 poseC.translation.x(), poseC.translation.y());
          goal = poseC;
          break;
      }
      displacement = goal.translation - current_pose.translation;
      break;
  }

  const double turn_angle = AngleMod(atan2(displacement.y(), displacement.x()));
  if (execution_state != WAIT) {
    goal.angle = turn_angle;
  }

  robot_logger->Pop();

  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  controller->SetGoal(goal);
  controller->Run();
}

void TriangleIdDependent::Reset() {
  const int& ssl_vision_id =
      world_state_.GetOurRobots().Get(our_robot_index_).ssl_vision_id;
  execution_state = WAIT;
  goal_pose = POSEA;
  poseA.Set(0.0f, Vector2f(1000 - 300 * ssl_vision_id, 1000));
  poseB.Set(0.0f, Vector2f(2500 - 300 * ssl_vision_id, 2000));
  poseC.Set(0.0f, Vector2f(3000 - 300 * ssl_vision_id, -2000));
  goal = poseA;

  wait_start_time_ = world_state_.world_time_;
  if (wait_start_time_ < GetWallTime()) {
    wait_start_time_ = GetWallTime();
  }
}

void TriangleIdDependent::Init() {
  const int& ssl_vision_id =
      world_state_.GetOurRobots().Get(our_robot_index_).ssl_vision_id;
  execution_state = WAIT;
  goal_pose = POSEA;
  poseA.Set(0.0f, Vector2f(1000 - 300 * ssl_vision_id, 1000));
  poseB.Set(0.0f, Vector2f(2500 - 300 * ssl_vision_id, 2000));
  poseC.Set(0.0f, Vector2f(3000 - 300 * ssl_vision_id, -2000));
  goal = poseA;

  wait_start_time_ = world_state_.world_time_;
  if (wait_start_time_ < GetWallTime()) {
    wait_start_time_ = GetWallTime();
  }
}

void TriangleIdDependent::SetGoal(const Pose2Df& pose) { goal = pose; }
}  // namespace tactics
