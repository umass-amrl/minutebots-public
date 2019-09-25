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

#include "tactics/three_kick.h"
#include <algorithm>
#include <cmath>
#include <iomanip>  // std::setprecision
#include <memory>
#include <utility>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "navigation/navigation_util.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "safety/dss2.h"
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

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Rotation2Df;
using geometry::Angle;
using geometry::RayIntersect;
using geometry::EuclideanDistance;
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
using obstacle::Obstacle;
using obstacle::ObstacleType;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;
using navigation::CollisionFreePath;
using navigation::ProjectToSafety;

namespace tactics {

ThreeKick::ThreeKick(const string& machine_name, const WorldState& world_state,
                     TacticArray* tactic_list, SharedState* shared_state,
                     OurRobotIndex our_robot_index,
                     state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name, world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      target_angle_(0),
      is_complete_(false),
      pass_only_(false),
      chip_(false),
      chip_distance_(0),
      kick_(std::bind(&ThreeKick::Kicking, this), "Kick"),
      post_kick_(std::bind(&ThreeKick::PostKick, this), "PostKick"),
      thresholds_ball_velocity_(100, 0.0, 5000.0, "ball_velocity", this),
      thresholds_distance_(kRotationRadius,  0.0, kFieldLength,
                           "distance", this),
      thresholds_kick_timeout_(40, 0.0, 100.0, "kick_timeout", this),
      thresholds_kick_percent_(50, 0.0, 100.0, "kick_percent", this),
      thresholds_kick_speed_(1000, 0.0, 5000.0,  "kick_speed", this),
      thresholds_follow_through_(10, 0.0, 100.0,  "kick_follow_through", this),
      kick_count_(0),
      target_set_(false) {
  state_ = kick_;
  kick_solution_.isInitialized = false;
}

void ThreeKick::Init() {}

void ThreeKick::Reset() {
  kick_solution_.isInitialized = false;
  state_ = kick_;
  is_complete_ = false;
  pass_only_ = false;
  kick_count_ = 0;
  target_set_ = false;
}

void ThreeKick::SetGoal(const Pose2Df& pose) {
  target_angle_ = pose.angle;
  target_set_ = true;
}

bool ThreeKick::IsComplete() { return is_complete_; }

float ThreeKick::GetCost() { return 0; }

void ThreeKick::SetPassOnly(const bool& pass_only) { pass_only_ = pass_only; }

void ThreeKick::SetChip(const bool& chip, const float& chip_distance) {
  chip_ = chip;
  chip_distance_ = chip_distance;
}

int ThreeKick::GetKickCount() {
  return kick_count_;
}


void ThreeKick::Kicking() {
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetDefenseAreas());
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  float omega =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.angle;
  kick_count_++;
  Eigen::Rotation2Df world_to_robot_rotation(-current_pose.angle);
  if (!target_set_) {
    OurRobotIndex target_robot;
    GetTarget(world_state_.GetBallPosition().position, world_state_,
              soccer_state_, our_robot_index_, pass_only_, &target_angle_,
              &target_robot);
  }

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrintPush("Kick");

  // Check if path to target is blocked
  if (shared_state_->IsPass() &&
      soccer_state_->GetRobotByOurRobotIndex(our_robot_index_)
              .current_tactic_ != tactics::KICKOFF) {
    Vector2f receive_position = kTheirGoalCenter;
    ObstacleFlag obstacles;
    obstacles =
        ObstacleFlag::GetAllRobotsExceptTeam(our_robot_index_);
    if (shared_state_->GetPassTarget() != offense::kNoPassRobot) {
      receive_position = shared_state_->GetPassLocation();
      obstacles = obstacles &
          ObstacleFlag::GetAllRobotsExceptTeam(shared_state_->GetPassTarget());
    }

    SafetyMargin safety_margin;

    std::pair<bool, const Obstacle*> is_collision_free_info =
        navigation::CollisionFreePathGetObstacle(obstacles, safety_margin,
                                                 current_pose.translation,
                                                 receive_position);
    if (!is_collision_free_info.first) {
      chip_ = true;
      const Vector2f obstacle_pose =
          is_collision_free_info.second->GetPose().translation;
      const Vector2f obstacle_distance =
          obstacle_pose - current_pose.translation;
      chip_distance_ = fabs(obstacle_distance.norm()) + kChipKickPadding;
    } else {
      chip_ = false;
    }
  } else {
    chip_ = false;
  }

  if (chip_) {
    state->chip_kick_set = true;
    robot_logger->LogPrint("Chipping");
    state->chip_kick = chip_distance_;
    shared_state_->SetPassShot();
  } else if (pass_only_) {
    state->flat_kick_set = true;
    robot_logger->LogPrint("Passing");
    if (!kRadioUsePassedPower) {
      state->flat_kick = 1.5;
    } else {
      const Vector2f robot_velocity =
          world_state_.GetOurRobotPosition(our_robot_index_)
              .velocity.translation;
      const float x_velocity = robot_velocity.x() / 1000;
      const float desired_speed = offense::kPassSpeed / 1000;
      const float kick_speed = desired_speed - x_velocity;
      robot_logger->LogPrint("Desired Kick: %f", desired_speed);
      robot_logger->LogPrint("x_velocity: %f", x_velocity);
      robot_logger->LogPrint("kick_speed: %f", kick_speed);
      state->flat_kick = kick_speed;
    }
    shared_state_->SetPassShot();
  } else {
    state->flat_kick_set = true;
    robot_logger->LogPrint("Shooting");
    if (!kRadioUsePassedPower) {
      state->flat_kick = 5.0;
    } else {
      const Vector2f robot_velocity =
          world_state_.GetOurRobotPosition(our_robot_index_)
              .velocity.translation;
      const float x_velocity = robot_velocity.x() / 1000;
      const float desired_speed = 6;
      const float kick_speed = desired_speed - 2 * x_velocity;
      robot_logger->LogPrint("Desired Kick: %f", desired_speed);
      robot_logger->LogPrint("x_velocity: %f", x_velocity);
      robot_logger->LogPrint("kick_speed: %f", kick_speed);
      state->flat_kick = kick_speed;
    }
    shared_state_->ClearPass();
  }
  robot_logger->Pop();

  // Try to hold angle to the desired angle
  ControlSequence1D rotational_control;
  TimeOptimalControlAnyFinal1D(current_pose.angle, omega, target_angle_, 0.0,
                               0.0, kMaxRobotRotAccel, kMaxRobotRotVel,
                               &rotational_control);

  float angular_accel = GetAccelToPreservePosition(
      rotational_control, kTransmitPeriodSeconds, omega);

  state->acceleration_command.angle = angular_accel;

  Vector2f target_vector(cos(target_angle_), sin(target_angle_));

  ControlSequence1D y_axis_control;
  const Vector2f y_dir(-sin(target_angle_), cos(target_angle_));
  const Vector2f robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f ball_distance = ball_pose - current_pose.translation;
  const float y_dist_from_intercept = y_dir.dot(ball_distance);

  const float target_velocity = 0;
  const float target_offset = 0;
  TimeOptimalControlAnyFinal1D(-y_dist_from_intercept, robot_velocity.y(),
                               target_offset, target_velocity, 0.0,
                               .4 * kMaxRobotAcceleration, kMaxRobotVelocity,
                               &y_axis_control);

  float y_accel = GetAccelToPreservePosition(
      y_axis_control, kTransmitPeriodSeconds, robot_velocity.y());

  float x_accel = .6 * kMaxRobotAcceleration;

  const Vector2f desired_accel(x_accel, y_accel);
  robot_logger->LogPrint("YDIST: %f", y_dist_from_intercept);
  robot_logger->LogPrint("Desired X: %f, desired_y: %f, current_angle %f",
                         x_accel, y_accel, current_pose.angle);
  state->acceleration_command.translation =
      Rotation2Df(current_pose.angle) * desired_accel;
}

void ThreeKick::PostKick() {
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
}

void ThreeKick::Transition() {
  // Setup
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);

  bool kicked = false;
  if (state_ == kick_) {
    // SET POTENTIAL TRANSITION
    potential_state_ = "POSTKICK";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;
    const float ball_target_projection = ball_vel.dot(Heading(target_angle_));
    const float ball_target_percent =
        100 * (ball_target_projection / ball_vel.norm());
    const Vector2f offset = ball_pose - current_pose.translation;
    const bool timed_out = kick_count_ > thresholds_kick_timeout_;
    kicked = fabs(ball_target_percent) > thresholds_kick_percent_ &&
             ball_vel.norm() > thresholds_kick_speed_ &&
             kick_count_ > thresholds_follow_through_ &&
             offset.norm() > thresholds_distance_;
    logger::Logger* robot_logger =
        soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
    if (!kicked) {
      if (ball_target_percent > thresholds_kick_percent_) {
        robot_logger->LogPrint("Kick percent Good");
      } else {
        robot_logger->LogPrint("Percent: %f, Thresh: %f", ball_target_percent,
                               static_cast<float>(thresholds_kick_percent_));
      }
      if (ball_vel.norm() > thresholds_kick_speed_) {
        robot_logger->LogPrint("Kick Speed Good");
      }
      if (kick_count_ > thresholds_follow_through_) {
        robot_logger->LogPrint("Kick Count Good");
      }
      if (offset.norm() > thresholds_distance_) {
        robot_logger->LogPrint("Kick Distance Good");
      }
    }
    SetTransition(timed_out);
    bool in_defense = false;
    if (soccer_state_->GetRobotByOurRobotIndex(our_robot_index_)
            .current_tactic_ != tactics::KICKOFF) {
      // Don't kick the ball if it is in their defense area.
      obstacle::ObstacleFlag flags =
          obstacle::ObstacleFlag::GetTheirDefenseArea();
      for (auto obstacle : flags) {
        if (obstacle->PointCollision(ball_pose, kBallRadius)) {
          in_defense = false;
        }
      }
    }
    if (timed_out || kicked || in_defense) {
      state_ = post_kick_;
      is_complete_ = true;
    }
  } else if (state_ == post_kick_) {
    Reset();
  }
}

}  // namespace tactics
