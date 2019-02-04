// Copyright 2017 - 2018 dbalaban@cs.umass.edu
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

#include "tactics/navigate_to_catch.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "navigation/navigation_util.h"
#include "obstacles/safety_margin.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/ntoc_controller.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Matrix2f;
using geometry::Perp;
using geometry::EuclideanDistance;
using math_util::AngleMod;
using geometry::SafeVectorNorm;
using navigation::ProjectToSafety;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using state::SharedRobotState;
using state::WorldState;
using obstacle::ObstacleFlag;
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
using tactics::EightGridNavigation;
using state::SharedState;
using offense::SetIsBallMotionObstacle;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;

namespace tactics {

NavigateToCatch::NavigateToCatch(const WorldState& world_state,
                                 TacticArray* tactic_list,
                                 SharedState* shared_state,
                                 OurRobotIndex our_robot_index,
                                 state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

NavigateToCatch::~NavigateToCatch() {}

Vector2f NavigateToCatch::GetNavigationPoint(Vector2f minimum, Vector2f maximum,
                                             unsigned int iterations) {
  Vector2f test_point = (minimum + maximum) / 2;
  MotionModel model(kDefaultRobotAcceleration, kDefaultRobotVelocity);
  ControlSequence2D linear_control;
  float robot_time = NTOC2D(test_point - current_pose.translation,
                            current_velocity_world, model, &linear_control);
  float ball_dist = (ball_pos - test_point).norm();
  float ball_time = robot_time;
  float r1 = 0;
  math_util::SolveQuadratic(-kBallAcceleration, ball_vel.norm(), ball_dist,
                            &ball_time, &r1);
  float spare_time = ball_time - robot_time;
  if (fabs(spare_time - kTimeBuffer_) < kTimeMargin_ ||
      iterations > kMaxIterations_) {
    return test_point;
  }

  if (spare_time < kTimeBuffer_) {
    return GetNavigationPoint(minimum, test_point, iterations + 1);
  }

  return GetNavigationPoint(test_point, maximum, iterations + 1);
}

float NavigateToCatch::FindCatchNavigationPoint(Pose2Df* target_pose) {
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  the_logger->LogPrint("Running Navigigate to Catch");
  the_logger->Push();

  current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  current_pose = world_state_.GetOurRobotPosition(our_robot_index_).position;
  ball_pos = world_state_.GetBallPosition().position;
  ball_vel = world_state_.GetBallPosition().velocity;
  ball_dir = ball_vel.normalized();

  the_logger->LogPrint("Colision Point: (%f, %f)", current_pose.translation.x(),
                       current_pose.translation.y());
  the_logger->LogPrint("Ball Pos: (%f, %f)", ball_pos.x(), ball_pos.y());
  the_logger->LogPrint("Distance: %f",
                       (ball_pos - current_pose.translation).norm());
  the_logger->LogPrint("Threshold: %f", kInterceptionRadius);

  the_logger->LogPrint(
      "Current Pose (%.5f, %.5f, %.5f)", current_pose.translation.x(),
      current_pose.translation.y(), RadToDeg(current_pose.angle));
  the_logger->LogPrint(
      "Current Velocity (%.5f, %.5f, %.5f)", current_velocity.translation.x(),
      current_velocity.translation.y(), RadToDeg(current_velocity.angle));

  the_logger->LogPrint("Current Ball Pose (%.5f, %.5f)", ball_pos.x(),
                       ball_pos.y());
  the_logger->LogPrint("Current Ball Velocity (%.5f, %.5f)", ball_vel.x(),
                       ball_vel.y(), RadToDeg(current_velocity.angle));

  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
  Eigen::Rotation2Df world_to_robot_rotation(-current_pose.angle);

  current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  the_logger->AddLine(
      current_pose.translation.x(), current_pose.translation.y(),
      current_pose.translation.x() + current_velocity_world.x() / 10.0,
      current_pose.translation.y() + current_velocity_world.y() / 10.0, 0, 1, 0,
      1);

  GetInterceptSolution(current_pose.translation.cast<double>(),
                       current_velocity_world.cast<double>(),
                       ball_pos.cast<double>(), ball_vel.cast<double>(),
                       kBallAcceleration, kMaxRobotAcceleration,
                       &intercept_solution);

  Vector2d final_robot_pos;
  Vector2d final_robot_vel;
  Vector2d final_ball_pos;
  Vector2d final_ball_vel;
  GetState(current_pose.translation.cast<double>(),
           current_velocity_world.cast<double>(), ball_pos.cast<double>(),
           ball_vel.cast<double>(), &final_robot_pos, &final_robot_vel,
           &final_ball_pos, &final_ball_vel, kMaxRobotAcceleration,
           intercept_solution.T, kBallAcceleration, intercept_solution);

  the_logger->AddCircle(final_ball_pos.cast<float>(), 0.9 * kBallRadius, 0.0,
                        0.0, 1.0, 1.0);
  the_logger->AddCircle(final_robot_pos.cast<float>(), 1.1 * kRobotRadius, 0.0,
                        1.0, 0.0, 1.0);
  if ((final_robot_pos - final_ball_pos).norm() >
      kInterceptionRadius + kInterceptionMargin) {
    Reset();
    the_logger->LogPrint("Found Point %f Distance from Target",
                         (final_robot_pos - final_ball_pos).norm());
    the_logger->LogPrint("Threshold is %f",
                         kInterceptionRadius + kInterceptionMargin);
  }

  Vector2f navigation_point =
      final_robot_pos.cast<float>() +
      kNavigationTimeAdjustment * final_ball_vel.cast<float>();
  const Vector2f ball_path = navigation_point - ball_pos;
  navigation_point = ball_pos + kNavigationDistAdjustment * ball_path;

  //   Vector2f ball_vel_normalized;
  //   if (final_ball_vel.x() == 0.0 && final_ball_vel.y() == 0.0) {
  //     ball_vel_normalized = final_ball_vel.cast<float>();
  //   } else {
  //     ball_vel_normalized = final_ball_vel.normalized().cast<float>();
  //   }
  //   Vector2f ball_accel =
  //       -kBallAcceleration * ball_vel_normalized;
  //   Vector2f max_bound = final_robot_pos.cast<float>()
  //       + final_ball_vel.cast<float>() * intercept_solution.T
  //       + 0.5 * ball_accel * Sq(intercept_solution.T);
  //   Vector2f navigation_point = GetNavigationPoint(ball_pos, max_bound, 0);

  navigation_point = ProjectToSafety(navigation_point, ball_pos,
                                     kAttackerFieldMargin, the_logger);
  target_pose->translation = navigation_point;
  target_pose->angle = AngleMod(atan2(-ball_dir.y(), -ball_dir.x()));

  MotionModel model(kDefaultRobotAcceleration, kDefaultRobotVelocity);
  ControlSequence2D linear_control;
  the_logger->Pop();
  return NTOC2D(navigation_point - current_pose.translation,
                current_velocity_world, model, &linear_control);
}

void NavigateToCatch::Run() {
  Pose2Df target_pose;
  FindCatchNavigationPoint(&target_pose);

  NTOC_Controller* controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  controller->TurnOnAngleRelaxation();

  EightGridNavigation* planner = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  ObstacleFlag obstacles = ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_);
  SetIsBallMotionObstacle(world_state_, our_robot_index_, &obstacles);
  planner->SetGoal(target_pose);
  planner->SetObstacles(obstacles);

  planner->Run();
}

void NavigateToCatch::Reset() { intercept_solution.isInitialized = false; }

void NavigateToCatch::Init() { intercept_solution.isInitialized = false; }

void NavigateToCatch::SetGoal(const pose_2d::Pose2Df& pose) {}

void NavigateToCatch::SetSolution(SolutionParameters solution) {
  intercept_solution = solution;
}

}  // namespace tactics
