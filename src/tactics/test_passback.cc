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

#include "tactics/test_passback.h"
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
#include "motion_control/ntoc_2d.h"
#include "obstacles/obstacle_flag.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/ball_interception.h"
#include "tactics/kick.h"

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
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;
using tactics::EightGridNavigation;

namespace tactics {

  TestPassback::TestPassback(const WorldState& world_state,
    TacticArray* tactic_list, SharedState* shared_state,
    OurRobotIndex our_robot_index, state::SoccerState* soccer_state)
    : StateMachineTactic("TestPassback", world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      wait_(std::bind(&TestPassback::Wait, this), "Wait"),
      receive_(std::bind(&TestPassback::Receive, this),
               "Receive"),
      pass_(std::bind(&TestPassback::Pass, this), "Pass"),
      complete_(false) {
  state_ = wait_;
}

void TestPassback::Init() {
}

void TestPassback::Reset() {
  state_ = wait_;
  Tactic* controller =
      (*tactic_list_)[TacticIndex::DEFLECTION].get();
  controller->Reset();
  controller =
      (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Reset();
  controller =
      (*tactic_list_)[TacticIndex::TEST_PASSING].get();
  controller->Reset();
}

void TestPassback::SetGoal(const Pose2Df& pose) {}

const Pose2Df TestPassback::GetGoal() { return robot_goal_pose; }

bool TestPassback::IsComplete() { return complete_; }

float TestPassback::GetCost() {
  return 0;
}

// Do nothing
void TestPassback::Wait() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;
  // Move to the wait positions
  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
    (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  // Run the controller with the calculated goal and margins.
  controller->SetGoal({current_pose.angle, setup_pose_});
  controller->Run();
}

void TestPassback::Receive() {
  Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Run();
}

void TestPassback::Pass() {
  Tactic* controller = (*tactic_list_)[TacticIndex::TEST_PASSING].get();
  controller->Run();
}

void TestPassback::Transition() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;
  if (first_) {
    setup_pose_ = current_pose.translation;
    first_ = false;
  }

  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const Vector2f ball_dist = current_pose.translation - ball_pose;
  const float ball_vel_threshold = 100;
  const float ball_dist_threshold = 5 * kRobotRadius;

  if (state_ == wait_) {
    // Decide whether to pass or receive (or keep waiting)
    if (shared_state_->IsPass() &&
        shared_state_->GetPassTarget() == our_robot_index_) {
      state_ = receive_;
    } else if (ball_vel.norm() < ball_vel_threshold &&
               ball_dist.norm() < ball_dist_threshold) {
      state_ = pass_;
    }
  } else if (state_ == receive_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
    if (controller->IsComplete()) {
      Reset();
      Transition();
    }
  } else if (state_ == pass_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::TEST_PASSING].get();
    if (controller->IsComplete()) {
      Reset();
      Transition();
    }
  }
  }

}  // namespace tactics
