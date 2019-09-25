// Copyright 2018 - 2019 jaholtz@cs.umass.edu, slane@cs.umass.edu
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

#include "tactics/better_ball_placement.h"
#include <algorithm>
#include <cmath>
#include <iomanip>  // std::setprecision
#include <memory>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/ntoc_2d.h"
#include "navigation/navigation_util.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/ball_interception.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/navigate_to_catch.h"
#include "tactics/navigate_to_intercept.h"
#include "tactics/ntoc_controller.h"
#include "safety/dss2.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Rotation2Df;
using geometry::Angle;
using geometry::RayIntersect;
using geometry::EuclideanDistance;
using geometry::ProjectPointOntoLine;
using state::WorldState;
using state::WorldRobot;
using state::SoccerRobot;
using state::SharedState;
using state::SharedRobotState;
using std::cos;
using std::sin;
using std::endl;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;
using tactics::InterceptionController;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using offense::GetBestChipTarget;
using offense::GetTarget;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using ntoc::GetAccelToPreservePosition;
using motion::MotionModel;
using math_util::Sign;
using navigation::CollisionFreePath;
using navigation::ProjectToSafety;

namespace tactics {

BetterBallPlacement::BetterBallPlacement(const WorldState& world_state,
            TacticArray* tactic_list,
            SharedState* shared_state,
            OurRobotIndex our_robot_index,
            state::SoccerState* soccer_state)
    : StateMachineTactic("Better Ball Placement", world_state, tactic_list,
                         shared_state, our_robot_index, soccer_state),
      robot_goal_pose(0, 0, 0),
      is_complete_(false),
      start_(std::bind(&BetterBallPlacement::Start, this), "Start"),
      setup_(std::bind(&BetterBallPlacement::Setup, this), "Setup"),
      place_(std::bind(&BetterBallPlacement::Place, this), "Place"),
      finished_(std::bind(&BetterBallPlacement::Finished, this), "Finished"),
      target_set_(false),
      direction_set_(false),
      place_start_(0, 0),
      target_(0, 0),
      thresholds_ball_velocity_(100, 0.0, 5000.0, "ball_velocity", this),
      thresholds_distance_(kRotationRadius, 0.0, kFieldLength,
                           "distance", this),
      kThresholdsX_(90, 0.0, kFieldLength, "x", this),
      kThresholdsY_(90, 0.0, kFieldLength, "y", this) {
  state_ = setup_;
}

void BetterBallPlacement::Init() {
}

void BetterBallPlacement::Reset() {
  state_ = start_;
  target_set_ = false;
  direction_set_ = false;
  place_start_ = {0, 0};
}

void BetterBallPlacement::SetGoal(const Pose2Df& pose) {
  robot_goal_pose = pose;
  target_set_ = true;
}

bool BetterBallPlacement::IsComplete() { return is_complete_; }

float BetterBallPlacement::GetCost() {
  Pose2Df current_pose =
  world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
  world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  MotionModel motion_model(kMaxRobotAcceleration, kMaxRobotVelocity);
  ControlSequence2D linear_control;

  Vector2f ntoc_position;
  Vector2f ntoc_velocity;

  ntoc::TransformCoordinatesForNTOC(current_pose, current_velocity.translation,
                                    ball_pose, &ntoc_position, &ntoc_velocity);

  ntoc_velocity.x() = 0;
  ntoc_velocity.y() = 0;

  return (NTOC2D(ntoc_position, ntoc_velocity, motion_model, &linear_control));
}

void BetterBallPlacement::Setup() {
  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
    (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
  controller->SetObstacles(
    ObstacleFlag::GetAllRobotsExceptTeam(our_robot_index_) |
    ObstacleFlag::GetBall());

  safety::DSS2::SetObstacleFlag(
    our_robot_index_,
    ObstacleFlag::GetAllRobotsExceptTeam(our_robot_index_) |
    ObstacleFlag::GetBall());

  NTOC_Controller* ntoc =
  static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());

  MotionModel slow_model(kMaxRobotAcceleration, 1000);

  ntoc->SetMotionModel(slow_model);
//   ntoc->TurnOffPDController();
  ntoc->TurnOffAngleRelaxation();
  controller->SetGoal(robot_goal_pose);

  controller->Run();
}


void BetterBallPlacement::Place() {
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->dribbler_set = true;
  state->dribbler_spin = 40;
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  float omega =
      world_state_.GetTheirRobotPosition(our_robot_index_).velocity.angle;

    logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  // Try to hold angle to the desired angle
  ControlSequence1D rotational_control;
  TimeOptimalControlAnyFinal1D(current_pose.angle,
                               omega,
                               robot_goal_pose.angle,
                               0.0,
                               0.0,
                               M_PI,
                               .1 * kMaxRobotRotVel,
                               &rotational_control);



  float angular_accel = GetAccelToPreservePosition(rotational_control,
                                                   kTransmitPeriodSeconds,
                                                   omega);

  state->acceleration_command.angle = angular_accel;

  const Vector2f ball_pose =
      world_state_.GetBallPosition().position;
  float robot_angle = current_pose.angle;
  const Vector2f line = robot_goal_pose.translation - ball_pose;
  float line_angle = Angle(line);

  const Vector2f robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).
          velocity.translation;
  const Vector2f relative_velocity =
      Rotation2Df(-line_angle) * Rotation2Df(robot_angle) * robot_velocity;

  Vector2f line_distance(0, 0);
  ControlSequence1D y_axis_control;

  robot_logger->LogPrint("Robot Goal Position: %f, %f",
                         robot_goal_pose.translation.x(),
                         robot_goal_pose.translation.y());

  Vector2f displacement = robot_goal_pose.translation -
                          current_pose.translation;
  robot_logger->LogPrint("Pre-Rotate Offset: %f",
                                                 displacement.x());
  robot_logger->LogPrint("Pre-Rotate Rotate: %f", displacement.y());
  robot_logger->AddLine(robot_goal_pose.translation,
                        ball_pose, 0, 0, 0, 1);
  displacement = Rotation2Df(-line_angle) * displacement;


  const float y_dist_from_intercept = displacement.y();

  const float target_velocity = 0;
  const float target_offset = 0;
  robot_logger->AddLine(current_pose.translation, displacement,
                        1.0,
                        1.0,
                        1.0,
                        1.0);
  TimeOptimalControlAnyFinal1D(-y_dist_from_intercept,
                               relative_velocity.y(),
                               target_offset,
                               target_velocity,
                               0.0,
                               .05 * kMaxRobotAcceleration,
                               250,
                               &y_axis_control);

  float y_accel = GetAccelToPreservePosition(y_axis_control,
                                             kTransmitPeriodSeconds,
                                             relative_velocity.y());

  const float x_dist_from_intercept = displacement.x();

  ControlSequence1D x_axis_control;

  TimeOptimalControlAnyFinal1D(-x_dist_from_intercept,
                               relative_velocity.x(),
                               target_offset,
                               target_velocity,
                               0.0,
                               0.5 * kMaxRobotAcceleration,
                               250,
                               &x_axis_control);

  float x_accel = GetAccelToPreservePosition(x_axis_control,
                                             kTransmitPeriodSeconds,
                                             relative_velocity.x());

  robot_logger->LogPrint("X Offset: %f", x_dist_from_intercept);
  robot_logger->LogPrint("Y Offset: %f", y_dist_from_intercept);

  robot_logger->LogPrint("Acceleration Line Frame: %f, %f, %f",
                         x_accel, y_accel, angular_accel);

  const Vector2f desired_accel(x_accel, y_accel);

  state->acceleration_command.translation =
      Rotation2Df(line_angle) * desired_accel;
}

void BetterBallPlacement::Start() {
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
}

void BetterBallPlacement::Finished() {
  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
    (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
  controller->SetObstacles(
    ObstacleFlag::GetAllRobotsExceptTeam(our_robot_index_) |
    ObstacleFlag::GetBall());

  safety::DSS2::SetObstacleFlag(
    our_robot_index_,
    ObstacleFlag::GetAllRobotsExceptTeam(our_robot_index_) |
    ObstacleFlag::GetBall());

  NTOC_Controller* ntoc =
  static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());

  MotionModel slow_model(kMaxRobotAcceleration, 1000);

  ntoc->SetMotionModel(slow_model);
  //   ntoc->TurnOffPDController();
  ntoc->TurnOffAngleRelaxation();
  controller->SetGoal(robot_goal_pose);

  controller->Run();
}


void BetterBallPlacement::Transition() {
  direction_set_ = false;
  const Pose2Df current_pose =
  world_state_.GetOurRobotPosition(our_robot_index_).position;

  const Pose2Df current_velocity =
  world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  logger::Logger* robot_logger =
  soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  Vector2f designated_place = {500, 500};
  if (soccer_state_->GetRefereeState().IsBallPlacementUs() &&
    soccer_state_->GetRefereeState().HasDesignatedPosition()) {
    designated_place = soccer_state_->GetRefereeState().GetDesignatedPostion();
    if (soccer_state_->direction_ == direction::NEGATIVE) {
      designated_place.x() = -designated_place.x();
      designated_place.y() = -designated_place.y();
    }
  } else if (target_set_) {
    designated_place = target_;
  }

  target_set_ = false;

  const Vector2f ball_position =
      world_state_.GetBallPosition().filtered_position;
  const Vector2f ball_displacement = designated_place - ball_position;
  if (state_ == start_) {
    direction_set_ = false;
    if (ball_displacement.norm() > 100) {
      state_ = setup_;
    } else {
      state_ = finished_;
    }
  } else if (state_ == setup_) {
    direction_set_ = false;
    EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

    robot_goal_pose = {
      Angle(ball_displacement),
      ball_position + (kRotationRadius * -ball_displacement.normalized())};
      controller->SetGoal(robot_goal_pose);
    controller->SetLocationThreshold(15.0f);
    if (controller->IsComplete()) {
      state_ = place_;
    }

    robot_logger->LogPrint("Goal: %f,%f,%f",
                            robot_goal_pose.translation.x(),
                            robot_goal_pose.translation.y(),
                            robot_goal_pose.angle);
    robot_logger->LogPrint("Position: %f,%f,%f", current_pose.translation.x(),
                           current_pose.translation.y(), current_pose.angle);
  } else if (state_ == place_) {
    // SET POTENTIAL TRANSITION
    potential_state_ = "Finished";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    if (!direction_set_) {
      robot_goal_pose.angle =
        Angle(Vector2f(designated_place - current_pose.translation));
      robot_goal_pose.translation =
        designated_place - kRobotFaceRadius * Heading(robot_goal_pose.angle);
      place_start_ = current_pose.translation;
    }

    const Vector2f goal_offset =
        robot_goal_pose.translation - current_pose.translation;
    const bool should_finish = fabs(goal_offset.x()) < kThresholdsX_ &&
                               fabs(goal_offset.y()) < kThresholdsY_;
    SetTransition(should_finish);
    if (should_finish) {
      state_ = start_;
    }
    const Vector2f ball_position = world_state_.GetBallPosition().position;
    const Vector2f displacement = ball_position - current_pose.translation;
    float target_angle = Angle(Vector2f(designated_place -
                                        current_pose.translation));
    if (displacement.norm() > kRobotRadius * 2 ||
      fabs(AngleDiff(current_pose.angle, target_angle)) > DegToRad(10.0)) {
      robot_logger->LogPrint("Displacement: %f", displacement.norm());
      Reset();
    }
  } else if (state_ == finished_) {
    direction_set_ = false;
    const Vector2f ball_position = world_state_.GetBallPosition().position;
    const Vector2f ball_displacement = ball_position - designated_place;
    if (ball_displacement.norm() > 100) {
      Reset();
    } else {
      robot_goal_pose = {Angle(Vector2f(kOurGoalCenter - ball_position)),
                         Vector2f(750 * (kOurGoalCenter - ball_position) +
                             ball_position)};
    }
  }
}

}  // namespace tactics
