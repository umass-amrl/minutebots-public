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

#include "tactics/navigate_to_intercept.h"

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
#include "navigation/navigation_util.h"
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
using geometry::EuclideanDistance;
using geometry::Perp;
using geometry::Angle;
using math_util::AngleMod;
using geometry::SafeVectorNorm;
using navigation::ProjectToSafety;
using state::SharedRobotState;
using state::WorldState;
using obstacle::ObstacleFlag;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
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
using offense::SetIsBallMotionObstacle;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;
using tactics::NTOC_Controller;

namespace tactics {

NavigateToIntercept::NavigateToIntercept(
    const WorldState& world_state, TacticArray* tactic_list,
    SharedState* shared_state, OurRobotIndex our_robot_index,
    state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

NavigateToIntercept::~NavigateToIntercept() {}

float NavigateToIntercept::FindInterceptNavigationPoint(Pose2Df* target_pose) {
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  the_logger->LogPrint("Running Navigigate to Ball Interception");
  the_logger->Push();

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pos = world_state_.GetBallPosition().position;
  Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  Vector2f desired_norm(cos(target_angle), sin(target_angle));
  // the balls center should be at (kRobotRadius + kBallRadius) * desired_norm
  // relative to the robot's center at collision time
  Vector2f collision_point =
      current_pose.translation + kInterceptionRadius * desired_norm;

  the_logger->LogPrint("Collision Point: (%f, %f)", collision_point.x(),
                       collision_point.y());
  the_logger->LogPrint("Ball Pos: (%f, %f)", ball_pos.x(), ball_pos.y());
  the_logger->LogPrint("Distance: %f", (ball_pos - collision_point).norm());
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

  Vector2f current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  the_logger->AddLine(
      current_pose.translation.x(), current_pose.translation.y(),
      current_pose.translation.x() + current_velocity_world.x() / 10.0,
      current_pose.translation.y() + current_velocity_world.y() / 10.0, 0, 1, 0,
      1);

  GetInterceptSolution(
      collision_point.cast<double>(), current_velocity_world.cast<double>(),
      ball_pos.cast<double>(), ball_vel.cast<double>(), kBallAcceleration,
      kMaxRobotAcceleration, &intercept_solution);

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

  // const Vector2f ball_dir = ball_vel.normalized();
  Vector2f navigation_point =
      final_robot_pos.cast<float>() + kNavigationTimeAdjustment * ball_vel;
  navigation_point = ProjectToSafety(
      navigation_point, ball_pos - kInterceptionRadius * desired_norm,
      kAttackerFieldMargin, the_logger);

  target_pose->translation = navigation_point;
  target_pose->angle = target_angle;

  MotionModel model(kDefaultRobotAcceleration, kDefaultRobotVelocity);
  ControlSequence2D linear_control;
  the_logger->Pop();
  return NTOC2D(navigation_point - current_pose.translation,
                current_velocity_world, model, &linear_control);
}

void NavigateToIntercept::Run() {
  Pose2Df target_pose;
  FindInterceptNavigationPoint(&target_pose);

  NTOC_Controller* controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  controller->TurnOnAngleRelaxation();

  EightGridNavigation* planner = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  planner->SetObstacles(obstacle::ObstacleFlag::GetAllExceptTeam(
          world_state_, *soccer_state_, our_robot_index_, our_robot_index_)
            | obstacle::ObstacleFlag::GetMediumBall());

  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pos = world_state_.GetBallPosition().position;

  zone::FieldZone field_zone(zone::FULL_FIELD);
  bool in_field = field_zone.IsInZone(
    ball_pos, kDefaultSafetyMargin);
  if (!in_field) {
    const Vector2f towards_ball = ball_pos - current_pose.translation;
    target_pose.angle = Angle(towards_ball);
  }

  planner->SetGoal(target_pose);
  planner->Run();
}

void NavigateToIntercept::Reset() { intercept_solution.isInitialized = false; }

void NavigateToIntercept::Init() { intercept_solution.isInitialized = false; }

void NavigateToIntercept::SetGoal(const pose_2d::Pose2Df& pose) {}

void NavigateToIntercept::SetAngle(float angle) { target_angle = angle; }

void NavigateToIntercept::SetSolution(SolutionParameters solution) {
  intercept_solution = solution;
}

}  // namespace tactics
