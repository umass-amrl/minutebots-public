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

#include "tactics/random_points.h"

#include <cmath>

#include <map>
#include <memory>

#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "logging/logger.h"
#include "math/poses_2d.h"
#include "navigation/navigation_util.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "util/random.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using navigation::PointCollision;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using state::WorldState;
using state::SharedState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;

namespace tactics {
RandomPoints::RandomPoints(const WorldState& world_state,
                           TacticArray* tactic_list,
                           SharedState* shared_state,
                           OurRobotIndex our_robot_index,
                           state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      goal(0, 0, 0),
      desired_pose(0, 0, 0) {}

void RandomPoints::Run() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_vel =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  // Tactic* controller = (*tactic_list_)[TacticIndex::STOX_NAVIGATION].get();
  // Tactic* controller = (*tactic_list_)[TacticIndex::NTOC].get();

  switch (execution_state) {
    case TURNING:
      robot_logger->LogPrint("Random Points: Turning");
      goal = current_pose;
      goal.angle = desired_pose.angle;
      controller = (*tactic_list_)[TacticIndex::NTOC].get();
      break;
    case FORWARD:
      robot_logger->LogPrint("Random Points: Going to %f, %f",
                             desired_pose.translation.x(),
                             desired_pose.translation.y());
      goal = desired_pose;
      break;
  }

  robot_logger->LogPrint("Goal: %f, %f, %f", goal.translation.x(),
                         goal.translation.y(), goal.angle);
  robot_logger->LogPrint("Current Pose: %f, %f, %f",
                         current_pose.translation.x(),
                         current_pose.translation.y(), current_pose.angle);
  robot_logger->LogPrint("Current Velocity: %f, %f, %f",
                         current_vel.translation.x(),
                         current_vel.translation.y(), current_vel.angle);

  if ((goal.translation - current_pose.translation).norm() <
          kDistanceThreshold &&
      std::abs(goal.angle - current_pose.angle) <
          kNavigationAngularThreshold * 3 &&
      current_vel.translation.norm() < kNavigationLocationThreshold * 5 &&
      fabs(current_vel.angle) < kNavigationAngularVelocityThreshold * 3) {
    switch (execution_state) {
      case TURNING:
        robot_logger->LogPrint("Switching to FORWARD");
        execution_state = FORWARD;
        break;
      case FORWARD:
        execution_state = TURNING;
        // Generate new point
        desired_pose.translation = GenerateSample();
        Vector2f displacement =
            desired_pose.translation - current_pose.translation;
        desired_pose.angle =
            AngleMod(atan2(displacement.y(), displacement.x()));
        break;
    }
  }

  controller->SetGoal(goal);
  controller->Run();
}

void RandomPoints::Reset() {
  execution_state = TURNING;
  goal.Set(0.0f, Vector2f(1000, 0));

  // Generate new point
  desired_pose.translation = GenerateSample();
}

void RandomPoints::Init() {
  execution_state = TURNING;
  goal.Set(0.0f, Vector2f(1000, 0));

  // Generate new point
  desired_pose.translation = GenerateSample();
}

void RandomPoints::SetGoal(const Pose2Df& pose) {}

Vector2f RandomPoints::GenerateSample() {
  Vector2f sample_point;
  SafetyMargin margin;

  margin.SetMargin(ObstacleType::ROBOT, kNavigationRobotMargin);

  static constexpr float kSizeReduction = 1500.0f;
  // Generate new point
  sample_point.x() =
      static_cast<float>(random.UniformRandom(0, kHalfFieldLength / 2 - 200));
  sample_point.y() = static_cast<float>(random.UniformRandom(
      -kHalfFieldWidth + kSizeReduction, kHalfFieldWidth - kSizeReduction));

  while (PointCollision(
      sample_point,
      ObstacleFlag::GetAll(world_state_, *soccer_state_, our_robot_index_),
      margin)) {
    // Generate new point
    sample_point.x() =
        static_cast<float>(random.UniformRandom(0, kHalfFieldLength / 2 - 200));
    sample_point.y() = static_cast<float>(random.UniformRandom(
        -kHalfFieldWidth + kSizeReduction, kHalfFieldWidth - kSizeReduction));
    //       LOG(WARNING) << "Here";
  }

  return sample_point;
}

}  // namespace tactics
