// Copyright 2018 jaholtz@cs.umass.edu
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

#include "tactics/ball_placement.h"
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
#include "motion_control/motion_model.h"
#include "motion_control/ntoc_2d.h"
#include "obstacles/obstacle_flag.h"
#include "safety/dss2.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/kick.h"
#include "tactics/ntoc_controller.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::RayIntersect;
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
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using offense::GetBestChipTarget;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using obstacle::ObstacleFlag;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;

namespace tactics {

BallPlacement::BallPlacement(const string& machine_name,
                             const WorldState& world_state,
                             TacticArray* tactic_list,
                             SharedState* shared_state,
                             OurRobotIndex our_robot_index,
                             state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name, world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      start_(std::bind(&BallPlacement::Start, this), "Start"),
      setup_(std::bind(&BallPlacement::Setup, this), "Setup"),
      place_(std::bind(&BallPlacement::Place, this), "Place"),
      finish_(std::bind(&BallPlacement::Finish, this), "Finish"),
      kThresholdsX_(20, 0.0, kFieldLength, "x", this),
      kThresholdsY_(20, 0.0, kFieldLength, "y", this),
      kThresholdsAngle_(DegToRad(0.5), 0.0, 3.14, "angle", this),
      complete_(false),
      target_set_(false),
      target_(0, 0) {
  state_ = start_;
}

void BallPlacement::Init() {}

void BallPlacement::Reset() {
  state_ = start_;
  target_set_ = false;
}

void BallPlacement::SetGoal(const Pose2Df& pose) {
  target_ = pose.translation;
  target_set_ = true;
}

const Pose2Df BallPlacement::GetGoal() {
  Vector2f designated_place = {500, 500};
  if (soccer_state_->GetRefereeState().IsBallPlacementUs() &&
      soccer_state_->GetRefereeState().HasDesignatedPosition()) {
    designated_place = soccer_state_->GetRefereeState().GetDesignatedPostion();
  } else if (target_set_) {
    designated_place = target_;
  }
  return {0, designated_place};
}

bool BallPlacement::IsComplete() { return complete_; }

// Cost is currently the ntoc time to the ball.
float BallPlacement::GetCost() {
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

void BallPlacement::Start() {
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->SetGoal(robot_goal_pose);
  controller->Run();
}

void BallPlacement::Setup() {
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

  MotionModel slow_model(kMaxRobotAcceleration, 500);

  ntoc->SetMotionModel(slow_model);
  controller->SetGoal(robot_goal_pose);

  controller->Run();
}

void BallPlacement::Place() {
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->dribbler_set = true;
  state->dribbler_spin = -10;
  NTOC_Controller* controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());

  MotionModel slow_model(kMaxRobotAcceleration, 150);
  controller->SetGoal(robot_goal_pose);
  controller->SetMotionModel(slow_model);
  controller->TurnOffPDController();
  controller->TurnOffAngleRelaxation();
  controller->Run();
}

void BallPlacement::Finish() {
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->SetGoal(robot_goal_pose);
  controller->Run();
}

void BallPlacement::Transition() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  robot_logger->LogPrint("Goal: %f,%f,%f", robot_goal_pose.translation.x(),
                         robot_goal_pose.translation.y(),
                         robot_goal_pose.angle);
  robot_logger->LogPrint("Position: %f,%f,%f", current_pose.translation.x(),
                         current_pose.translation.y(), current_pose.angle);

  Vector2f designated_place = {500, 500};
  if (soccer_state_->GetRefereeState().IsBallPlacementUs() &&
      soccer_state_->GetRefereeState().HasDesignatedPosition()) {
    designated_place = soccer_state_->GetRefereeState().GetDesignatedPostion();
  } else if (target_set_) {
    designated_place = target_;
  }

  target_set_ = false;

  const Vector2f ball_position = world_state_.GetBallPosition().position;
  const Vector2f ball_displacement = designated_place - ball_position;
  if (state_ == start_) {
    if (ball_displacement.norm() > 100) {
      state_ = setup_;
    } else {
      state_ = finish_;
    }
  } else if (state_ == setup_) {
    EightGridNavigation* controller = static_cast<EightGridNavigation*>(
        (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

    robot_goal_pose = {
        Angle(ball_displacement),
        ball_position + (kRotationRadius * -ball_displacement.normalized())};
    controller->SetGoal(robot_goal_pose);
    if (controller->IsComplete()) {
      state_ = place_;
    }

  } else if (state_ == place_) {
    // SET POTENTIAL TRANSITION
    potential_state_ = "Finish";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    robot_goal_pose.angle =
        Angle(Vector2f(designated_place - current_pose.translation));
    robot_goal_pose.translation =
        designated_place - kRobotFaceRadius * Heading(robot_goal_pose.angle);

    const Vector2f goal_offset =
        robot_goal_pose.translation - current_pose.translation;
    const bool should_finish = fabs(goal_offset.x()) < kThresholdsX_ &&
                               fabs(goal_offset.y()) < kThresholdsY_;
    SetTransition(should_finish);
    if (should_finish) {
      state_ = finish_;
    }
    const Vector2f ball_position = world_state_.GetBallPosition().position;
    const Vector2f displacement = ball_position - current_pose.translation;
    if (displacement.norm() > kRobotRadius * 4) {
      robot_logger->LogPrint("Displacement: %f", displacement.norm());
      Reset();
    }
  } else if (state_ == finish_) {
    const Vector2f ball_position = world_state_.GetBallPosition().position;
    const Vector2f ball_displacement = ball_position - designated_place;
    if (ball_displacement.norm() > 100) {
      Reset();
    }
  }
}

}  // namespace tactics
