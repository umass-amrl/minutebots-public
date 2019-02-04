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

#include "tactics/primary_defender.h"

#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/defense_evaluation.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "obstacles/safety_margin.h"
#include "radio_protocol_wrapper.pb.h"
#include "safety/dss2.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/ntoc_controller.h"


#define _USE_MATH_DEFINES

STANDARD_USINGS;
using defense::DefenseEvaluator;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using obstacle::ObstacleType;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using pose_2d::Pose2Df;
using state::WorldState;
using state::SharedState;
using std::atan2;
using std::endl;
using std::map;
using std::tan;
using std::unique_ptr;
using tactics::EightGridNavigation;
using tactics::NTOC_Controller;
using tactics::TacticIndex;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;

namespace tactics {

PrimaryDefender::PrimaryDefender(const WorldState& world_state,
                                 TacticArray* tactic_list,
                                 SharedState* shared_state,
                                 OurRobotIndex our_robot_index,
                                 state::SoccerState* soccer_state)
    : StateMachineTactic("PrimaryDefender",
                         world_state,
                         tactic_list,
                         shared_state,
                         our_robot_index,
                         soccer_state),
      guard_(std::bind(&PrimaryDefender::Guard, this), "Guard"),
      intercept_(std::bind(&PrimaryDefender::Intercept, this), "Intercept"),
      kMaxInterceptCount_(10, "InterceptCount", this),
      kBallKickThreshold_(250.0f, "KickSpeedThreshold", this),
      intercepting_(false) {
  state_ = guard_;
  guard_pose_ = pose_2d::Pose2Df(0, 0, 0);
}

void PrimaryDefender::Init() {
  guard_pose_ = pose_2d::Pose2Df(0, 0, 0);
}

void PrimaryDefender::Guard() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Guard Point: %f, %f",
                         guard_pose_.translation.x(),
                         guard_pose_.translation.y());

  float current_speed = world_state_.GetOurRobotPosition(our_robot_index_).
                            velocity.translation.norm();

  ObstacleFlag obstacles = ObstacleFlag::GetAllExceptTeam(world_state_,
                                                          *soccer_state_,
                                                          our_robot_index_,
                                                          our_robot_index_);
  if (current_speed < kIgnoreOpponentsThreshold_ ||
      soccer_state_->IsNormalPlay()) {
    obstacles = obstacles & (~ObstacleFlag::GetOpponentRobots());
  }
  Navigate(obstacles);
}

void PrimaryDefender::Intercept() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Intercept Point: %f, %f",
                         guard_pose_.translation.x(),
                         guard_pose_.translation.y());

  float current_speed = world_state_.GetOurRobotPosition(our_robot_index_).
                            velocity.translation.norm();

  ObstacleFlag obstacles = ObstacleFlag::GetAllExceptTeam(world_state_,
                                                          *soccer_state_,
                                                          our_robot_index_,
                                                          our_robot_index_);
  obstacles = obstacles & ~ObstacleFlag::GetAllBalls();
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacles);

  if (current_speed < kIgnoreOpponentsThreshold_) {
    obstacles = obstacles & (~ObstacleFlag::GetOpponentRobots());
  }

  Navigate(obstacles);

  // This is updated by the Defense Evaluator
  intercepting_ = false;
}

void PrimaryDefender::Transition() {
  if (state_ == guard_) {
    potential_state_ = "INTERCEPT";
    AddBlock(true);
    and_clause_ = true;

    if (intercepting_) {
      SetTransition(true);
      state_ = intercept_;
    } else {
      SetTransition(false);
    }
  } else if (state_ == intercept_) {
    potential_state_ = "GUARD";
    AddBlock(true);
    and_clause_ = true;

    if (!intercepting_) {
      SetTransition(true);
      state_ = guard_;
    } else {
      SetTransition(false);
    }
  }
}

void PrimaryDefender::Reset() {
  if (world_state_.GetOurRobotPosition(our_robot_index_).confidence > 0) {
    Pose2Df current_pose =
        world_state_.GetOurRobotPosition(our_robot_index_).position;

    guard_pose_ = pose_2d::Pose2Df(0, 0, 0);
  } else {
    guard_pose_ = pose_2d::Pose2Df(0, 0, 0);
  }

  state_ = guard_;
  intercepting_ = false;
}

void PrimaryDefender::SetGoal(const Pose2Df& pose) { guard_pose_ = pose; }

bool PrimaryDefender::IsComplete() {
  return false;
}


float PrimaryDefender::GetCost() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  Vector2f current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;

  current_velocity = Rotation2Df(current_pose.angle) * current_velocity;

  return DefenseEvaluator::CalculatePrimaryCost(current_pose.translation,
                                                current_velocity);
}

void PrimaryDefender::SetIntercept() {
  intercepting_ = true;
}

void PrimaryDefender::Navigate(const ObstacleFlag& obstacles) {
  NTOC_Controller* controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  controller->TurnOnAngleRelaxation();


  EightGridNavigation* planner = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  logger::Logger* robot_logger =
  soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Navigation Goal: %f, %f",
                         guard_pose_.translation.x(),
                         guard_pose_.translation.y());

  Pose2Df target_pose;
  planner->SetGoal(guard_pose_);
  planner->SetObstacles(obstacles);
  planner->Run();
}


}  // namespace tactics
