// Copyright 2018 kvedder@umass.edu
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

#include "tactics/stox_pivot.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "safety/dss2.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/ntoc_controller.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using geometry::Perp;
using math_util::AngleMod;
using geometry::SafeVectorNorm;
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
using std::vector;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;
using ntoc::ControlPhase1D;
using ntoc::ControlSequence1D;
using ntoc::ControlPhase2D;
using motion::MotionModel;
using math_util::Sign;

namespace tactics {

STOXPivot::STOXPivot(const WorldState& world_state, TacticArray* tactic_list,
                     SharedState* shared_state, OurRobotIndex our_robot_index,
                     state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      goal_(0, 0, 0) {}

const Pose2Df STOXPivot::GetGoal() { return goal_; }

Vector2f CloserToGoal(const Vector2f p1, const Vector2f p2,
                      const Pose2Df& goal) {
  if ((goal.translation - p1).squaredNorm() <
      (goal.translation - p2).squaredNorm()) {
    return p1;
  }
  return p2;
}

bool LineToGoalFree(const Vector2f& p, const Vector2f& goal,
                    const obstacle::Obstacle& obstacle,
                    const float safety_margin) {
  return !obstacle.LineCollision(p, goal, safety_margin);
}

Vector2f GetRobotBallTangent(const Vector2f& current_position,
                             const Vector2f& ball_position,
                             const obstacle::Obstacle& ball_obstacle,
                             const float safety_margin, const Pose2Df& goal) {
  Vector2f right_tangent(0, 0);
  Vector2f left_tangent(0, 0);
  geometry::GetTangentPoints(current_position, ball_position,
                             ball_obstacle.GetRadius() + safety_margin,
                             &right_tangent, &left_tangent);

  // Tangent along which we want to take the path to the goal.
  return CloserToGoal(right_tangent, left_tangent, goal);
}

void STOXPivot::NTOCToPoint(const Pose2Df& point) {
  NTOC_Controller* ntoc_controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  ntoc_controller->TurnOnAngleRelaxation();
  ntoc_controller->SetGoal(point);
  ntoc_controller->Run();
}

Vector2f GetGoalBallTangent(const Vector2f& goal_position,
                            const Vector2f& ball_position) {
  const Vector2f ball_to_goal_delta = (goal_position - ball_position);
  return geometry::Perp(ball_to_goal_delta);
}

Vector2f GetCurrentTangentGoalTangentIntersect(const Vector2f& current_position,
                                               const Vector2f& current_tangent,
                                               const Vector2f& goal_position,
                                               const Vector2f& goal_tangent) {
  return geometry::LineLineIntersection(current_position, current_tangent,
                                        goal_position, goal_tangent);
}

void STOXPivot::Run() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  logger->Push("Running STOXPivot");
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetDefenseAreas());

  const Pose2Df current_robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const Vector2f current_ball_pose = world_state_.GetBallPosition().position;

  const obstacle::Obstacle& small_ball_obstacle =
      **(obstacle::ObstacleFlag::GetBall().begin());
  const float safety_margin =
      obstacle::SafetyMargin().GetMargin(obstacle::ObstacleType::BALL);

  if (LineToGoalFree(current_robot_pose.translation, goal_.translation,
                     small_ball_obstacle, safety_margin)) {
    NTOCToPoint(goal_);
    logger->AddCircle(goal_.translation, 30, 0, 1, 1, 1);
    logger->AddLine(
        goal_.translation,
        goal_.translation +
            Vector2f(std::cos(goal_.angle), std::sin(goal_.angle)) * 30,
        0, 1, 1, 1);
    logger->LogPrint("Straight to goal");
    logger->Pop();
    return;
  }

  const Vector2f robot_to_ball_tangent =
      GetRobotBallTangent(current_robot_pose.translation, current_ball_pose,
                          small_ball_obstacle, safety_margin, goal_);
  const Vector2f goal_to_ball_tangent =
      GetGoalBallTangent(goal_.translation, current_ball_pose);

  const Vector2f tangent_intersect = GetCurrentTangentGoalTangentIntersect(
      current_robot_pose.translation, robot_to_ball_tangent, goal_.translation,
      goal_to_ball_tangent);

  NTOCToPoint({goal_.angle, tangent_intersect});
  logger->LogPrint("Going to tangent intersect");
  logger->AddCircle(tangent_intersect, 30, 0, 1, 1, 1);
  logger->AddLine(
      tangent_intersect,
      tangent_intersect +
          Vector2f(std::cos(goal_.angle), std::sin(goal_.angle)) * 30,
      0, 1, 1, 1);
  logger->Pop();
}

void STOXPivot::Reset() {}

void STOXPivot::Init() {}

void STOXPivot::SetGoal(const pose_2d::Pose2Df& pose) { goal_ = pose; }

}  // namespace tactics
