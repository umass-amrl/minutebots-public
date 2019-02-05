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

#include "tactics/goalie.h"

#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/defense_evaluation.h"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "radio_protocol_wrapper.pb.h"
#include "safety/dss2.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/ball_placement.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/indirect_free_kicker.h"

#define _USE_MATH_DEFINES

STANDARD_USINGS;
using defense::DefenseEvaluator;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using geometry::Angle;
using geometry::Perp;
using geometry::LineLineIntersection;
using geometry::ProjectPointOntoLineSegment;
using geometry::RayIntersect;
using geometry::EuclideanDistance;
using geometry::SafeVectorNorm;
using geometry::FurthestFreePointCircle;
using math_util::Sign;
using offense::AimOption;
using offense::CalculateAimOptions;
using pose_2d::Pose2Df;
using state::WorldState;
using state::WorldRobot;
using state::SoccerRobot;
using state::SharedState;
using std::atan2;
using std::endl;
using std::map;
using std::tan;
using std::min;
using std::max;
using std::unique_ptr;
using tactics::EightGridNavigation;
using tactics::TacticIndex;

namespace tactics {

// Set goalie static members
SSLVisionId Goalie::goalie_ssl_id_ = 42;
bool Goalie::goalie_set_ = false;

Goalie::Goalie(const WorldState& world_state, TacticArray* tactic_list,
               SharedState* shared_state, OurRobotIndex our_robot_index,
               state::SoccerState* soccer_state)
    : StateMachineTactic("Goalie", world_state, tactic_list, shared_state,
                         our_robot_index, soccer_state),
      guard_(std::bind(&Goalie::Guard, this), "Guard"),
      intercept_(std::bind(&Goalie::Intercept, this), "Intercept"),
      dive_setup_(std::bind(&Goalie::DiveSetup, this), "DiveSetup"),
      dive_(std::bind(&Goalie::Dive, this), "Dive"),
      //       move_ball_(std::bind(&Goalie::MoveBall, this), "MoveBall"),
      clear_(std::bind(&Goalie::Clear, this), "Clear"),
      threat_position_(0, 0),
      guard_point_((kOurGoalL + kOurGoalR) / 2.0f),
      previous_intercept_point_((kOurGoalL + kOurGoalR) / 2.0f),
      previous_goal_point_(guard_point_),
      ball_placement_target_(0, guard_point_),
      set_ball_placement_target_(false),
      dive_counter_(0),
      intercept_counter_(0),
      kMaxInterceptCount_(25, 0.0, 100.0, "InterceptCount", this),
      kMaxDiveCount_(100, 0.0, 1000.0, "DiveCount", this),
      kBallKickThreshold_(250.0f, 0.0, 5000.0, "KickSpeedThreshold", this),
      kMoveBallThresholdX_(300.0f, 0.0, 5000.0, "MoveBallThresholdX", this),
      kClearThresholdSpeed_(100.0f, 0.0, 5000.0,
                            "kClearThresholdSpeed_", this) {
  state_ = guard_;
  previous_state_ = guard_;
}

void Goalie::Init() {}

void Goalie::Transition() {
  const Vector2f current_ball_observation =
      world_state_.GetBallPosition().observed_pose;

  Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  float ball_speed = SafeVectorNorm(ball_velocity);

  if (state_ != dive_setup_ && soccer_state_->IsTheirPenaltyKick()) {
    potential_state_ = "DIVESETUP";
    AddBlock(true);
    and_clause_ = true;
    SetTransition(true);
    state_ = dive_setup_;
  } else if (ShouldDive()) {
    state_ = dive_;
  } else if (ShouldIntercept(ball_pose, ball_velocity, current_ball_observation,
                             ball_speed)) {
    state_ = intercept_;
    //   } else if (ShouldMove(ball_pose, ball_velocity, ball_speed)) {
    //     state_ = move_ball_;
  } else if (ShouldClear(ball_pose, ball_velocity, ball_speed)) {
    state_ = clear_;
  } else if (state_ != dive_setup_) {
    potential_state_ = "GUARD";
    AddBlock(true);
    and_clause_ = false;

    AddBlock(true);
    and_clause_ = true;
    SetTransition(true);
    state_ = guard_;
  }

  // Reset items that need resetting

  //   if (previous_state_ == move_ball_ && state_ != move_ball_) {
  //     set_ball_placement_target_ = false;
  //     BallPlacement* controller = static_cast<BallPlacement*>(
  //         (*tactic_list_)[TacticIndex::BALL_PLACEMENT].get());
  //
  //     controller->Reset();
  //   } else

  if (previous_state_ == clear_ && state_ != clear_) {
    IndirectFreeKicker* controller = static_cast<IndirectFreeKicker*>(
        (*tactic_list_)[TacticIndex::INDIRECT_FREE_KICKER].get());
    controller->Reset();
  } else if (previous_state_ == dive_ && state_ != dive_) {
    dive_counter_ = 0;
  } else if (previous_state_ == intercept_ && state_ != intercept_) {
    intercept_counter_ = 0;
  }

  previous_state_ = state_;
}

void Goalie::Guard() {
  // Goalie has been run. Try to stick with this goalie
  Goalie::goalie_set_ = true;

  Pose2Df robot_goal_pose;

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  float threat_angular_width;
  bool default_position =
      CalculateGuardPosition(&guard_point_, &threat_angular_width);

  float threat_distance = EuclideanDistance(threat_position_, guard_point_);

  if (threat_position_.x() < -kFieldXMax) {
    robot_logger->LogPrint("Ball outside field, blocking same point");
    robot_goal_pose.translation = previous_goal_point_;
  } else if (default_position) {
    robot_logger->LogPrint("Full angle is blocked by defenders, using default");
    robot_goal_pose.translation = guard_point_;
  } else if (CanBlockFullAngle(threat_distance, threat_angular_width,
                               guard_point_, robot_logger,
                               &robot_goal_pose.translation)) {
    robot_logger->LogPrint("Can block full angle");
  } else {
    robot_logger->LogPrint("Default Guard Mode");
    robot_logger->LogPrint("Guard Point: %f, %f", guard_point_.x(),
                           guard_point_.y());
    float target_distance_from_goal =
        kGoalieRatio * threat_distance / (1.0f + kGoalieRatio);

    Vector2f threat_direction = threat_position_ - guard_point_;
    Vector2f position = target_distance_from_goal * threat_direction /
                            SafeVectorNorm(threat_direction) +
                        guard_point_;

    if (EuclideanDistance(position, kOurGoalCenter) > kGoalieMaxDistance) {
      float sq_distance;
      FurthestFreePointCircle(threat_position_, guard_point_, kOurGoalCenter,
                              kGoalieMaxDistance, &sq_distance, &position);

      Vector2f direction = position - kOurGoalCenter;
      direction.normalize();
      position = direction * kGoalieMaxDistance + kOurGoalCenter;
    }

    robot_goal_pose.translation = position;
  }

  robot_goal_pose.angle =
      Angle<float>(threat_position_ - robot_goal_pose.translation);

  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());

  previous_goal_point_ = robot_goal_pose.translation;
  Navigate(robot_goal_pose);
}

void Goalie::Intercept() {
  Vector2f target_point;
  float target_distance;

  Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  Vector2f current_position =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;

  Vector2f ball_intersect_point;
  float ball_intersect_distance;
  bool ball_heading_towards_goal =
      RayIntersect(ball_pose, ball_velocity, kOurGoalL, kOurGoalR,
                   &ball_intersect_distance, &ball_intersect_point);

  Pose2Df robot_goal_pose;

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->AddCircle(target_point, 20, 0, 0, 0, 0.75);

  if (ball_heading_towards_goal) {
    intercept_counter_ = 0;

    ProjectPointOntoLineSegment(current_position, ball_pose,
                                ball_intersect_point, &target_point,
                                &target_distance);

    robot_goal_pose.translation = target_point;
    robot_goal_pose.angle = Angle<float>(ball_pose - target_point);
    previous_intercept_point_ = target_point;
  } else {
    robot_goal_pose.translation = previous_intercept_point_;
    robot_goal_pose.angle = Angle<float>(ball_pose - previous_intercept_point_);
    intercept_counter_++;
  }

  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());

  Navigate(robot_goal_pose);
}

void Goalie::DiveSetup() {
  Pose2Df robot_goal_pose;
  robot_goal_pose.translation = kOurGoalCenter;
  robot_goal_pose.translation.x() += kRobotRadius;
  robot_goal_pose.angle = 0.0f;

  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());

  Navigate(robot_goal_pose);
}

void Goalie::Dive() {
  Intercept();
  return;
}

// void Goalie::MoveBall() {
//   if (!set_ball_placement_target_) {
//     Vector2f ball_pose = world_state_.GetBallPosition().position;
//     ball_placement_target_ =
//         Pose2Df(0,
//                 -kFieldXMax + kEpsilon +
//                     static_cast<float>(kMoveBallThresholdX_) + 100.0,
//                 ball_pose.y());
//     set_ball_placement_target_ = true;
//   }
//
//   BallPlacement* controller = static_cast<BallPlacement*>(
//       (*tactic_list_)[TacticIndex::BALL_PLACEMENT].get());
//   safety::DSS2::SetObstacleFlag(our_robot_index_,
//                                 obstacle::ObstacleFlag::GetBall());
//   controller->SetGoal(ball_placement_target_);
//   controller->Run();
// }

void Goalie::Clear() {
  // Just call indirect free kicker
  IndirectFreeKicker* controller = static_cast<IndirectFreeKicker*>(
      (*tactic_list_)[TacticIndex::INDIRECT_FREE_KICKER].get());
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetBall());
  controller->SetIsGoalie();
  controller->Run();
}

void Goalie::Navigate(Pose2Df robot_goal_pose) {
  obstacle::ObstacleFlag flags = obstacle::ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_);

  obstacle::ObstacleFlag ball_flag = obstacle::ObstacleFlag::GetBall();
  ball_flag = ~ball_flag.GetBall();

  flags = flags & ball_flag;

  Navigate(robot_goal_pose, flags);
}

void Goalie::Navigate(Pose2Df robot_goal_pose, obstacle::ObstacleFlag flags) {
  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

  controller->SetObstacles(flags);
  controller->SetGoal(robot_goal_pose);
  controller->Run();

  previous_goal_point_ = robot_goal_pose.translation;
}

// bool Goalie::ShouldMove(const Vector2f& ball_pose,
//                         const Vector2f& ball_velocity,
//                         float ball_speed) {
//   potential_state_ = "MOVEBALL";
//   AddBlock(true);
//   and_clause_ = true;
//
//   bool slow_ball = ball_speed < kClearThresholdSpeed_;
//   bool close_ball = ball_pose.x() + kFieldXMax < kMoveBallThresholdX_;
//
//   if (slow_ball &&
//       close_ball &&
//       fabs(ball_pose.y()) <= kDefenseStretch &&
//       ball_pose.x() >= -kFieldXMax &&
//       soccer_state_->IsNormalPlay()) {
//     SetTransition(true);
//     return true;
//   } else {
//     SetTransition(false);
//     return false;
//   }
// }

bool Goalie::ShouldClear(const Vector2f& ball_pose,
                         const Vector2f& ball_velocity, float ball_speed) {
  if (state_ == dive_ || state_ == dive_setup_ ||
      soccer_state_->IsTheirPenaltyKick()) {
    return false;
  }

  potential_state_ = "CLEAR";
  AddBlock(true);
  and_clause_ = true;

  bool slow_ball = ball_speed < kClearThresholdSpeed_;

  if (slow_ball && fabs(ball_pose.y()) <= kDefenseStretch &&
      ball_pose.x() <= -kFieldXMax + kDefenseStretch &&
      ball_pose.x() >= -kFieldXMax && soccer_state_->IsNormalPlay()) {
    SetTransition(true);
    return true;
  } else {
    SetTransition(false);
    return false;
  }
}

bool Goalie::ShouldDive() {
  if (state_ == dive_setup_) {
    potential_state_ = "DIVE";
    AddBlock(true);
    and_clause_ = true;
    SetTransition(!soccer_state_->IsTheirPenaltyKick());

    if (!soccer_state_->IsTheirPenaltyKick()) {
      return true;
    } else {
      return false;
    }
  } else if (state_ == dive_) {
    potential_state_ = "DIVE";
    AddBlock(true);
    and_clause_ = true;
    dive_counter_++;
    bool not_timed_out = dive_counter_ < kMaxDiveCount_;
    SetTransition(not_timed_out);

    if (!not_timed_out) {
      dive_counter_ = 0;
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

bool Goalie::ShouldIntercept(const Vector2f& ball_pose,
                             const Vector2f& ball_velocity,
                             const Vector2f& ball_observation,
                             float ball_speed) {
  potential_state_ = "INTERCEPT";
  AddBlock(true);
  if (state_ != intercept_) {
    and_clause_ = true;

    bool ball_heading_towards_goal =
        DefenseEvaluator::BallMayEnterGoal(ball_pose, ball_velocity, true);
    bool fast_ball = ball_speed > kBallKickThreshold_;

    SetTransition(ball_heading_towards_goal && fast_ball);
    if (ball_heading_towards_goal && fast_ball)
      return true;
    else
      return false;
  } else {
    and_clause_ = false;
    bool timed_out = intercept_counter_ > kMaxInterceptCount_;
    bool ball_out_of_bounds = ball_observation.x() < -kFieldXMax;

    SetTransition(!(timed_out || ball_out_of_bounds));
    if (timed_out || ball_out_of_bounds) {
      return false;
    } else {
      return true;
    }
  }
}

bool Goalie::CalculateGuardPosition(Vector2f* goalie_target,
                                    float* threat_angular_width) {
  static const bool kDebug = false;
  Vector2f target = kOurGoalCenter;
  target.x() += 2 * kRobotRadius;
  float angle_width = M_PI - kEpsilon;
  bool default_position = true;

  if (soccer_state_->GetRefereeState().IsStop()) {
    guard_point_ = kOurGoalCenter;
    guard_point_.x() += kRobotRadius;
    return true;
  }

  if (threat_position_.x() > -kFieldXMax) {
    // Calculate aim options with no obstacles
    vector<AimOption> aim_options;

    logger::Logger* robot_logger =
        soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
    if (kDebug) {
      robot_logger->LogPrint("Primary threat at: %f, %f", threat_position_.x(),
                             threat_position_.y());
      for (const Vector2f& position : defender_positions_) {
        robot_logger->LogPrint("Defender at: %f, %f", position.x(),
                               position.y());
      }
    }

    CalculateAimOptions(threat_position_, kOurGoalL, kOurGoalR, kBallRadius,
                        kRobotRadius, defender_positions_, &aim_options);

    if (!aim_options.empty()) {
      int max_width_index = -1;
      float max_width = -1;
      int current_index = 0;
      for (const AimOption& option : aim_options) {
        if (option.angle_width > max_width) {
          max_width = option.angle_width;
          max_width_index = current_index;
        }
        //         if (kDebug) {
        //           robot_logger->AddLine(threat_position_,
        //                                   option.target_l,
        //                                   1, 1, 1, 1);
        //           robot_logger->AddLine(threat_position_,
        //                                   option.target_r,
        //                                   1, 1, 1, 1);
        //         }
        current_index++;
      }
      angle_width = max_width;
      target = aim_options[max_width_index].target_bisector;
      default_position = false;

      if (kDebug) {
        robot_logger->LogPrint("Found %d aim options", aim_options.size());
        robot_logger->LogPrint("Ran with %d defenders",
                               defender_positions_.size());
      }
    }
  }
  defender_positions_.clear();

  *goalie_target = target;
  *threat_angular_width = angle_width;
  return default_position;
}

bool Goalie::CanBlockFullAngle(float threat_distance,
                               float threat_angular_width,
                               const Vector2f& guard_position,
                               logger::Logger* logger,
                               Vector2f* goalie_position) {
  static const bool kDebug = false;
  float full_block_distance_from_threat;
  full_block_distance_from_threat =
      kRobotRadius / tan(threat_angular_width / 2.0);

  float target_distance = threat_distance - full_block_distance_from_threat;
  Vector2f direction = threat_position_ - guard_position;
  direction.normalize();
  *goalie_position = direction * target_distance + guard_position;
  if (kDebug) {
    logger->LogPrint("Threat distance %f", threat_distance);
    logger->LogPrint("Target distance %f", target_distance);
    logger->AddLine(threat_position_, guard_position, 0, 0, 0, 1);
  }

  return EuclideanDistance(kOurGoalCenter, *goalie_position) <=
         kGoalieMaxDistance;
}

bool Goalie::IsComplete() { return false; }

void Goalie::Reset() {
  state_ = guard_;
  previous_state_ = guard_;
  intercept_counter_ = 0;
  dive_counter_ = 0;
  set_ball_placement_target_ = false;
  BallPlacement* placer = static_cast<BallPlacement*>(
      (*tactic_list_)[TacticIndex::BALL_PLACEMENT].get());

  placer->Reset();

  IndirectFreeKicker* kicker = static_cast<IndirectFreeKicker*>(
      (*tactic_list_)[TacticIndex::INDIRECT_FREE_KICKER].get());
  kicker->Reset();
}

void Goalie::SetGoal(const Pose2Df& pose) {
  threat_position_ = pose.translation;
}

float Goalie::GetCost() {
  if (world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id ==
      goalie_ssl_id_) {
    return 0.0f;
  } else {
    return 10e20;
  }
}

void Goalie::AddDefender(OurRobotIndex index) {
  defender_positions_.push_back(
      world_state_.GetOurRobotPosition(index).position.translation);
}

}  // namespace tactics
