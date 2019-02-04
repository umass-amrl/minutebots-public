// Copyright 2018 slane@cs.umass.edu, kvedder@umass.edu
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

#include "tactics/custom_route.h"

#include <cmath>

#include <fstream>
#include <map>
#include <memory>
#include <string>

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

CustomRoute::CustomRoute(const WorldState& world_state,
                         TacticArray* tactic_list,
                         SharedState* shared_state,
                         OurRobotIndex our_robot_index,
                         state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

void CustomRoute::Run() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  robot_logger->LogPrint("CustomRoute");
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
    ++goal_pose;
    if (goal_pose >= pose_list.size()) {
      goal_pose = 0;
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
      displacement =
          pose_list[goal_pose].translation - current_pose.translation;
      break;
    case FORWARD:
      robot_logger->LogPrint("Going Forward");
      goal = pose_list[goal_pose];
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

void CustomRoute::Reset() { Init(); }

void CustomRoute::Init() {
  if (pose_list.empty()) {
    const std::string file_path = "experimental/custom_routes/route" +
                                  std::to_string(Tactic::our_robot_index_) +
                                  ".txt";
    std::ifstream infile(file_path);

    if (!infile) {
      LOG(FATAL) << "Cannot open file " << file_path;
    }

    std::string line;
    while (std::getline(infile, line)) {
      std::istringstream iss(line);
      float x = 0, y = 0;
      if (!(iss >> x >> y)) {
        break;
      }

      pose_list.push_back({0, x, y});
    }

    if (!kProduction && pose_list.empty()) {
      LOG(FATAL) << "Pose list empty!";
    }
  }

  if (pose_list.empty()) {
    LOG(FATAL) << "No poses to initialize!";
  }

  execution_state = WAIT;
  goal_pose = 0;
  goal = pose_list[goal_pose];

  wait_start_time_ = world_state_.world_time_;
  if (wait_start_time_ < GetWallTime()) {
    wait_start_time_ = GetWallTime();
  }
}

void CustomRoute::SetGoal(const Pose2Df& pose) { goal = pose; }
}  // namespace tactics
