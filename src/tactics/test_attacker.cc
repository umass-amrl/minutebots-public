// Copyright 2018 slane@cs.umass.edu
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

#include "tactics/test_attacker.h"

#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <string>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/defense_evaluation.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/motion_model.h"
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
using geometry::Angle;
using geometry::ProjectPointOntoLine;
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
using std::fclose;
using std::FILE;
using std::fopen;
using std::fprintf;

namespace tactics {
bool TestAttacker::dump_file_opened_ = false;
FILE* TestAttacker::file_ = NULL;

TestAttacker::TestAttacker(const WorldState& world_state,
                           TacticArray* tactic_list,
                           SharedState* shared_state,
                           OurRobotIndex our_robot_index,
                           state::SoccerState* soccer_state)
    : StateMachineTactic("TestAttacker",
                         world_state,
                         tactic_list,
                         shared_state,
                         our_robot_index,
                         soccer_state),
      guard_(std::bind(&TestAttacker::Guard, this), "Guard"),
      intercept_(std::bind(&TestAttacker::Intercept, this), "Intercept"),
      intercepting_(false) {
  state_ = guard_;
  guard_pose_ = pose_2d::Pose2Df(0, 0, 0);

  if (kDumpData_) {
    string dump_file_name = "test_attacker.csv";
    file_ = fopen(dump_file_name.c_str(), "w");
  }
}

void TestAttacker::Init() {
  guard_pose_ = pose_2d::Pose2Df(0, 0, 0);
}

void TestAttacker::Guard() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  Vector2f ball_pose = world_state_.GetBallPosition().position;

  guard_pose_.translation = -kTheta_ * (kTheirGoalCenter -
                                        ball_pose).normalized()
                            + ball_pose;

  robot_logger->LogPrint("Guard Point: %f, %f",
                         guard_pose_.translation.x(),
                         guard_pose_.translation.y());

  ObstacleFlag obstacles = ObstacleFlag::GetAllExceptTeam(world_state_,
                                                          *soccer_state_,
                                                          our_robot_index_,
                                                          our_robot_index_);
  Navigate(obstacles, guard_pose_.translation);
  DumpData();
}

void TestAttacker::Intercept() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  Vector2f current_position =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;

  Vector2f target_point;

  ProjectPointOntoLine(current_position,
                       ball_pose,
                       Vector2f(ball_velocity + ball_pose),
                       &target_point);

  robot_logger->LogPrint("Intercept Point: %f, %f",
                         target_point.x(),
                         target_point.y());

  guard_pose_.angle = Angle(Vector2f(ball_pose - target_point));

  ObstacleFlag obstacles = ObstacleFlag::GetAllExceptTeam(world_state_,
                                                          *soccer_state_,
                                                          our_robot_index_,
                                                          our_robot_index_);
  obstacles = obstacles & ~ObstacleFlag::GetAllBalls();

  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacles);

  Navigate(obstacles, target_point);

  // This is updated by the Defense Evaluator
  intercepting_ = false;
  DumpData();
}

void TestAttacker::Transition() {
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

void TestAttacker::Reset() {
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

void TestAttacker::SetGoal(const Pose2Df& pose) { guard_pose_ = pose; }

bool TestAttacker::IsComplete() {
  return false;
}


float TestAttacker::GetCost() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  Vector2f current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;

  current_velocity = Rotation2Df(current_pose.angle) * current_velocity;

  return DefenseEvaluator::CalculateTertiaryCost(
      current_pose.translation,
      current_velocity);
}

void TestAttacker::SetIntercept() {
  intercepting_ = true;
}

void TestAttacker::Navigate(const ObstacleFlag& obstacles,
                                 const Vector2f& target_point) {
  NTOC_Controller* controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  controller->TurnOnAngleRelaxation();
  controller->SetMotionModel(motion::MotionModel(1000.0f,
                                                 kMaxRobotAcceleration));


  EightGridNavigation* planner = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  planner->SetGoal(Pose2Df(guard_pose_.angle, target_point));
  planner->SetObstacles(obstacles);
  planner->Run();
}

void TestAttacker::DumpData() {
  if (kDumpData_) {
    double observed_time =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_time;
    Vector2f ball_observation =
      world_state_.GetBallPosition().observed_pose;
    Vector2f robot_observation =
        world_state_.GetOurRobotPosition(our_robot_index_)
                    .observed_pose.translation;

    fprintf(file_,
            "%f, %f, %f, %f, %f\n",
            observed_time,
            ball_observation[0],
            ball_observation[1],
            robot_observation[0],
            robot_observation[1]);
  }
}


}  // namespace tactics
