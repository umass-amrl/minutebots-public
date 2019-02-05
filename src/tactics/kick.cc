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

#include "tactics/kick.h"
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

STANDARD_USINGS;
using Eigen::Rotation2Df;
using Eigen::Vector2d;
using Eigen::Vector2f;
using geometry::Angle;
using geometry::EuclideanDistance;
using geometry::RayIntersect;
using geometry::SafeVectorNorm;
using math_util::Sign;
using motion::MotionModel;
using navigation::CollisionFreePath;
using navigation::ProjectToSafety;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using obstacle::Obstacle;
using obstacle::ObstacleFlag;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestChipTarget;
using offense::GetBestPassTarget;
using offense::GetTarget;
using pose_2d::Pose2Df;
using state::SharedRobotState;
using state::SharedState;
using state::SoccerRobot;
using state::WorldRobot;
using state::WorldState;
using std::cos;
using std::endl;
using std::map;
using std::sin;
using std::unique_ptr;
using tactics::InterceptionController;
using tactics::TacticIndex;

namespace tactics {

Kick::Kick(const string& machine_name,
           const WorldState& world_state,
           TacticArray* tactic_list,
           SharedState* shared_state,
           OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name,
                         world_state,
                         tactic_list,
                         shared_state,
                         our_robot_index,
                         soccer_state),
      target_angle_(0),
      is_complete_(false),
      pass_only_(false),
      kick_(std::bind(&Kick::Kicking, this), "Kick"),
      post_kick_(std::bind(&Kick::PostKick, this), "PostKick"),
      thresholds_ball_velocity_(100, 0.0, 5000.0, "ball_velocity", this),
      thresholds_distance_(kRotationRadius, 0.0, kFieldLength,
                           "distance", this),
      thresholds_kick_timeout_(40, 0.0, 100.0, "kick_timeout", this),
      thresholds_kick_percent_(50, 0.0, 100.0, "kick_percent", this),
      thresholds_kick_speed_(1000, 0.0, 5000.0, "kick_speed", this),
      thresholds_follow_through_(10, 0.0, 100.0, "kick_follow_through", this),
      kick_count_(0),
      target_set_(false) {
  state_ = kick_;
  kick_solution_.isInitialized = false;
}

void Kick::Init() {}

void Kick::Reset() {
  kick_solution_.isInitialized = false;
  state_ = kick_;
  is_complete_ = false;
  pass_only_ = false;
  kick_count_ = 0;
  target_set_ = false;
}

void Kick::SetGoal(const Pose2Df& pose) {
  target_angle_ = pose.angle;
  target_set_ = true;
}

bool Kick::IsComplete() { return is_complete_; }

float Kick::GetCost() { return 0; }

void Kick::SetPassOnly(const bool& pass_only) { pass_only_ = pass_only; }

void Kick::Kicking() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetDefenseAreas());
  kick_count_++;
  state->flat_kick_set = true;
  const Vector2f robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_to_robot_displace = robot_pose - ball_pose;
  ball_to_robot_displace.normalize();
  const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  const float relative_ball_vel = ball_to_robot_displace.dot(ball_velocity);
  Eigen::Rotation2Df world_to_robot_rotation(-current_pose.angle);
  InterceptionController* controller = static_cast<InterceptionController*>(
      (*tactic_list_)[TacticIndex::BALL_INTERCEPTION].get());
  if (!target_set_) {
    OurRobotIndex target_robot;
    GetTarget(world_state_.GetBallPosition().position,
              world_state_,
              soccer_state_,
              our_robot_index_,
              pass_only_,
              &target_angle_,
              &target_robot);
  }
  bool chip = false;
  // Check if path to target is blocked
  if (shared_state_->IsPass()) {
    Vector2f receive_position = kTheirGoalCenter;
    ObstacleFlag obstacles;
    obstacles = ObstacleFlag::GetAllRobotsExceptTeam(our_robot_index_);
    if (shared_state_->GetPassTarget() != offense::kNoPassRobot) {
      receive_position = shared_state_->GetPassLocation();
      NP_CHECK_MSG(shared_state_->GetPassTarget() < kMaxTeamRobots,
                   "Pass target ("
                       << shared_state_->GetPassTarget()
                       << ") larger than max num robots (" << kMaxTeamRobots
                       << ") "
                       << " Location: " << shared_state_->GetPassLocation().x()
                       << " < " << shared_state_->GetPassLocation().y()
                       << " Command time: " << shared_state_->GetCommandTime());
      obstacles = obstacles & ObstacleFlag::GetAllRobotsExceptTeam(
                                  shared_state_->GetPassTarget());
    }

    SafetyMargin safety_margin;

    std::pair<bool, const Obstacle*> is_collision_free_info =
        navigation::CollisionFreePathGetObstacle(obstacles,
                                                 safety_margin,
                                                 current_pose.translation,
                                                 receive_position);
    if (!is_collision_free_info.first) {
      chip = true;
    } else {
      chip = false;
    }
  }
  if (chip) {
    state->chip_kick_set = true;
    state->chip_kick = 2000;
    shared_state_->SetPassShot();
  } else if (pass_only_) {
    if (!kRadioUsePassedPower) {
      state->flat_kick = 1.5;
    } else {
      const Vector2f robot_velocity =
          world_state_.GetOurRobotPosition(our_robot_index_)
              .velocity.translation;
      float x_velocity = robot_velocity.x() / 1000;
      x_velocity += relative_ball_vel / 1000;
      const float desired_speed = 2.5;
      const float kick_speed = desired_speed - x_velocity;
      state->flat_kick = kick_speed;
    }
    shared_state_->SetPassShot();
  } else {
    if (!kRadioUsePassedPower) {
      state->flat_kick = 5;
    } else {
      const Vector2f robot_velocity =
          world_state_.GetOurRobotPosition(our_robot_index_)
              .velocity.translation;
      float x_velocity = robot_velocity.x() / 1000;
      x_velocity += .2 * (relative_ball_vel / 1000);
      const float desired_speed = 6;
      const float kick_speed = desired_speed - x_velocity;
      state->flat_kick = kick_speed;
    }
    shared_state_->ClearPass();
  }
  controller->SetAngle(target_angle_);
  controller->SetSolution(kick_solution_);
  // have the robot intercept the ball so that the ball touches it on its face
  Vector2f offset_dir(cos(target_angle_), sin(target_angle_));
  Vector2f offset = kRobotFaceRadius * offset_dir;
  controller->SetOffset(offset);
  controller->match_velocity_ = false;
  controller->TurnOffAngleRelaxation();
  controller->Run();
  kick_solution_ = controller->GetSolution();
  // if the ball is very close to the face of the robot,
  // command extra x velocity (ie move forward)
  Vector2f ball_distance = world_state_.GetBallPosition().position -
                           (current_pose.translation + offset);
  float ball_distance_x = ball_distance.dot(offset_dir);
  Vector2f offset_orthogonal_dir(-sin(target_angle_), cos(target_angle_));
  float ball_distance_y = ball_distance.dot(offset_orthogonal_dir);
  if (ball_distance_x < 3 * kBallRadius &&
      fabs(ball_distance_y) < 2 * kBallRadius &&
      fabs(ball_velocity.norm()) >
          static_cast<float>(thresholds_ball_velocity_)) {
    logger::Logger* robot_logger =
        soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
    robot_logger->LogPrint("Commanding extra x velocity.");

    Vector2f translational_accel = state->acceleration_command.translation;
    translational_accel += Rotation2Df(current_pose.angle) *
                           Vector2f(0.9 * kMaxRobotAcceleration, 0);
    float accel_magnitude = SafeVectorNorm(translational_accel);

    if (accel_magnitude > kMaxRobotAcceleration) {
      translational_accel.normalize();
      translational_accel *= kMaxRobotAcceleration;
    }

    state->acceleration_command.translation = translational_accel;
  }
}

void Kick::PostKick() {
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
}

void Kick::Transition() {
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
        robot_logger->LogPrint("Percent: %f, Thresh: %f",
                               ball_target_percent,
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
    // Don't kick the ball if it is in their defense area.
    obstacle::ObstacleFlag flags =
        obstacle::ObstacleFlag::GetTheirDefenseArea();
    bool in_defense = false;
    for (auto obstacle : flags) {
      if (obstacle->PointCollision(ball_pose, kBallRadius)) {
        in_defense = true;
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
