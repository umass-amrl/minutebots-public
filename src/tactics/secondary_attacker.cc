// Copyright 2017 - 2018 slane@cs.umass.edu, jaholtz@cs.umass.edu
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

#include "tactics/secondary_attacker.h"

#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "navigation/navigation_util.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/eight_grid_navigation.h"
#include "obstacles/obstacle_flag.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using geometry::EuclideanDistance;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
using state::WorldState;
using state::WorldRobot;
using state::SharedState;
using state::SharedRobotState;
using std::cos;
using std::sin;
using std::endl;
using std::map;
using std::unique_ptr;
using tactics::EightGridNavigation;
using tactics::TacticIndex;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using geometry::Angle;

namespace tactics {
  SecondaryAttacker::SecondaryAttacker(const WorldState& world_state,
                             TacticArray* tactic_list,
                             SharedState* shared_state,
                             OurRobotIndex our_robot_index,
                             state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

             void SecondaryAttacker::Init() {
  robot_goal_pose.Set(0.0f, Vector2f(1000, 1000));
  target_angle = 0.0f;
  target_position.x() = 0;
  target_position.y() = 0.0f;
}

// Cost is currently the distance to the ball.
float SecondaryAttacker::GetCost() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f current_trans = current_pose.translation;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  float distance = sqrt(pow(current_trans.x() - ball_pose.x(), 2) +
                        pow(current_trans.y() - ball_pose.y(), 2));
  return distance;
}

void SecondaryAttacker::Run() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Secondary Attacker");
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  // Move to a position on the opposite side of the ball from the goal and wait.
  const Vector2f goal_to_ball = ball_pose - kTheirGoalCenter;
  const Vector2f ball_to_goal = kTheirGoalCenter - ball_pose;

  robot_goal_pose.translation = ball_pose + 1500 * goal_to_ball.normalized();
  robot_goal_pose.angle = Angle(ball_to_goal);
  // Call the controllers.
  EightGridNavigation* controller =
        static_cast<EightGridNavigation*>(
          (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
  obstacle::ObstacleFlag flags =
      obstacle::ObstacleFlag::GetAllExceptTeam(world_state_,
                                                *soccer_state_,
                                                our_robot_index_,
                                                our_robot_index_);

  controller->SetObstacles(flags);
  controller->SetGoal(robot_goal_pose);
  controller->Run();
}

void SecondaryAttacker::Reset() {
  robot_goal_pose.Set(0.0f, Vector2f(1000, 1000));
  target_angle = 0.0f;
  target_position.x() = 0;
  target_position.y() = 0.0f;
}

void SecondaryAttacker::SetGoal(const Pose2Df& pose) {}

bool SecondaryAttacker::IsComplete() { return true; }

}  // namespace tactics
