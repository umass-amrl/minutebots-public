// Copyright 2018 - 2019 jaholtz@cs.umass.edu
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

#include "tactics/state_machine_example.h"
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

namespace tactics {

StateMachineExample::StateMachineExample(
    const string& machine_name, const WorldState& world_state,
    TacticArray* tactic_list, SharedState* shared_state,
    OurRobotIndex our_robot_index, state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name, world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      start_(std::bind(&StateMachineExample::Start, this), "Start"),
      drive_(std::bind(&StateMachineExample::Drive, this), "Drive"),
      rotate_(std::bind(&StateMachineExample::Rotate, this),
                 "Rotate"),
      drive2_(std::bind(&StateMachineExample::Drive2, this), "Drive2"),
      finish_(std::bind(&StateMachineExample::Finish, this), "Finish"),
      kThresholdsX_(5, 0.0, 8000.0, "x", this),
      kThresholdsY_(5, 0.0, 8000.0,  "y", this),
      kThresholdsAngle_(DegToRad(0.5), 0.0, 3.14, "angle", this) {
  state_ = start_;
}

void StateMachineExample::Init() {
}

void StateMachineExample::Reset() {
}

void StateMachineExample::SetGoal(const Pose2Df& pose) {}

const Pose2Df StateMachineExample::GetGoal() { return robot_goal_pose; }

bool StateMachineExample::IsComplete() { return false; }

// Cost is currently the ntoc time to the ball.
float StateMachineExample::GetCost() {
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

void StateMachineExample::Start() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  state->velocity_x = 0;
  state->velocity_y = 0;
  state->velocity_r = 1;
}

void StateMachineExample::Drive() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  state->velocity_x = 1000.0;
  state->velocity_y = 0;
  state->velocity_r = 0.0;
}

void StateMachineExample::Drive2() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = 1000.0;
  state->velocity_y = 0.0;
  state->velocity_r = 0.0;
}

void StateMachineExample::Rotate() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = 0;
  state->velocity_y = 0;
  state->velocity_r = -1;
}

void StateMachineExample::Finish() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = 0;
  state->velocity_y = 0;
  state->velocity_r = 10.0;
}

void StateMachineExample::Transition() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  robot_logger->LogPrint("Goal: %f,%f,%f", robot_goal_pose.translation.x(),
                          robot_goal_pose.translation.y(),
                          robot_goal_pose.angle);
  robot_logger->LogPrint("Position: %f,%f,%f", current_pose.translation.x(),
                         current_pose.translation.y(),
                         current_pose.angle);

  if (state_ == start_) {
    robot_goal_pose = current_pose;
    robot_goal_pose.angle = 3.14;
    const float angle_offset = AngleDiff(current_pose.angle,
                                       robot_goal_pose.angle);
    // SET POTENTIAL TRANSITION
    potential_state_ = "DRIVE";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;
    logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
    robot_logger->LogPrint("Angle Diff: %f", angle_offset);
    const bool should_drive = fabs(angle_offset) < kThresholdsAngle_;

    SetTransition(should_drive);
    if (should_drive) {
      robot_goal_pose = {0, 0, 0};
      robot_goal_pose.translation.x() -= 500;
      robot_goal_pose.angle = 3.14;
      state_ = drive_;
      robot_logger->LogPrint("Goal: %f,%f,%f", robot_goal_pose.translation.x(),
                          robot_goal_pose.translation.y(),
                          robot_goal_pose.angle);
    }
  } else if (state_ == drive_) {
    // SET POTENTIAL TRANSITION
    potential_state_ = "ROTATE";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    const Vector2f goal_offset =
      robot_goal_pose.translation - current_pose.translation;

    robot_logger->LogPrint("Offset: %f,%f", goal_offset.x(),
                          goal_offset.y());

    const bool should_rotate = fabs(goal_offset.x()) < kThresholdsX_;

    SetTransition(should_rotate);

    if (should_rotate) {
      state_ = rotate_;
    }
  } else if (state_ ==  rotate_) {
    robot_goal_pose = current_pose;
    robot_goal_pose.angle = 1.5708;
    // SET POTENTIAL TRANSITION
    potential_state_ = "DRIVE2";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    const float angle_offset = AngleDiff(current_pose.angle,
                                       robot_goal_pose.angle);
    const bool should_drive2 = fabs(angle_offset) < kThresholdsAngle_;
    SetTransition(should_drive2);
    if (should_drive2) {
      state_ = drive2_;
      robot_goal_pose.translation.y() += 500;
    }
  } else if (state_ == drive2_) {
    // SET POTENTIAL TRANSITION
    potential_state_ = "FINISH";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;

    const Vector2f goal_offset =
      robot_goal_pose.translation - current_pose.translation;

    robot_logger->LogPrint("Offset: %f,%f", goal_offset.x(),
                        goal_offset.y());

    const bool should_finish = fabs(goal_offset.x()) < kThresholdsX_
        && fabs(goal_offset.y())  < kThresholdsY_;
    SetTransition(should_finish);
    if (should_finish) {
      state_ = finish_;
    }
  }
}

}  // namespace tactics
