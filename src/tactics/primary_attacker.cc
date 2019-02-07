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

#include "tactics/primary_attacker.h"
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
#include "tactics/deflection.h"
#include "tactics/eight_grid_navigation.h"
#include "tactics/kick.h"
#include "tactics/navigate_to_catch.h"
#include "tactics/navigate_to_intercept.h"
#include "tactics/ntoc_controller.h"
#include "tactics/receiver.h"
#include "tactics/stox_pivot.h"
#include "tactics/three_kick.h"

STANDARD_USINGS;
using Eigen::Vector2d;
using Eigen::Vector2f;
using geometry::Angle;
using geometry::CheckLineLineIntersection;
using geometry::EuclideanDistance;
using geometry::ProjectPointOntoLine;
using geometry::RayIntersect;
using math_util::Sign;
using math_util::SolveQuadratic;
using motion::MotionModel;
using navigation::CollisionFreePath;
using navigation::ProjectToSafety;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using obstacle::ObstacleFlag;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestChipTarget;
using offense::GetBestPassTarget;
using offense::GetTargetEvaluated;
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
using tactics::Deflection;
using tactics::InterceptionController;
using tactics::Kick;
using tactics::NTOC_Controller;
using tactics::Receiver;
using tactics::TacticIndex;
using tactics::ThreeKick;
using srtr::StateMachine;

namespace tactics {

PrimaryAttacker::PrimaryAttacker(const string& machine_name,
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
      last_target_(offense::kNoPassRobot),
      ball_intercept_point_(Vector2f(0, 0)),
      set_kick_goal(false),
      chip_(false),
      chip_distance_(0),
      is_complete_(false),
      should_pass_(false),
      intercepting_(false),
      aim_count_(0),
      kMaxAimIterations(3),
      start_(std::bind(&PrimaryAttacker::Start, this), "Start"),
      catch_(std::bind(&PrimaryAttacker::Catch, this), "Catch"),
      intercept_(std::bind(&PrimaryAttacker::Intercept, this), "Intercept"),
      kick_(std::bind(&PrimaryAttacker::Kick, this), "Kick"),
      post_kick_(std::bind(&PrimaryAttacker::PostKick, this), "PostKick"),
      navigate_to_catch_(std::bind(&PrimaryAttacker::NavigateCatch, this),
                         "NavigateToCatch"),
      navigate_to_intercept_(
          std::bind(&PrimaryAttacker::NavigateIntercept, this),
          "NavigateToIntercept"),
      receive_(std::bind(&PrimaryAttacker::Receive, this), "Receive"),
      stox_pivot_(std::bind(&PrimaryAttacker::STOXPivot, this), "STOXPivot"),
      thresholds_angle_(1.5, 0.0, 360.0, "angle", this),
      thresholds_distance_(kRotationRadius, 0.0, kFieldLength,
                           "distance", this),
      thresholds_y_prime_vel_(200, 0.0, 5000.0, "y_prime_vel", this),
      thresholds_relative_vel_(400, 0.0, 5000.0, "relative_vel", this),
      thresholds_align_(18, 0.0, kFieldLength, "align", this),
      thresholds_angular_vel_(50, 0.0, 5000.0, "angular_vel", this),
      thresholds_kick_timeout_(40, 0.0, 100.0, "kick_timeout", this),
      thresholds_lower_catch_velocity_(1500, 0.0, 5000.0,
                                       "catch_velocity", this),
      thresholds_could_score_speed_(1000, 0.0, 5000.0,
                                    "could_score_speed", this),
      thresholds_ball_velocity_(100, 0.0, 5000.0, "ball_velocity", this),
      thresholds_catch_radius_(6 * kRobotRadius, 0.0, kFieldLength,
                               "catch_radius", this),
      thresholds_kick_percent_(85, 0, 100, "kick_percent", this),
      thresholds_kick_speed_(1000, 0.0, 5000.0, "kick_speed", this),
      thresholds_follow_through_(10, 0, 100, "kick_follow_through", this),
      kick_count_(0),
      prep_count_(0),
      last_target_score_(10.0),
      last_target_robot_(42),
      last_target_position_({0, 0}) {
  state_ = start_;
  LoadConfigFile();
}

void PrimaryAttacker::SetValue(nlohmann::json config_json,
                               RepairableParam* param) {
  string name = param->name_;
  auto it = config_json.find(name);
  if (it != config_json.end()) {
    param->SetValue(config_json[name].get<float>());
  }
}

void PrimaryAttacker::LoadConfigFile() {
  std::ifstream json_file("src/configs/attacker_config.json");
  nlohmann::json config_json;
  json_file >> config_json;
  SetValue(config_json, &thresholds_angle_);
  SetValue(config_json, &thresholds_distance_);
  SetValue(config_json, &thresholds_y_prime_vel_);
  SetValue(config_json, &thresholds_relative_vel_);
  SetValue(config_json, &thresholds_align_);
  SetValue(config_json, &thresholds_angular_vel_);
  SetValue(config_json, &thresholds_kick_timeout_);
  SetValue(config_json, &thresholds_lower_catch_velocity_);
  SetValue(config_json, &thresholds_could_score_speed_);
  SetValue(config_json, &thresholds_ball_velocity_);
  SetValue(config_json, &thresholds_catch_radius_);
  SetValue(config_json, &thresholds_kick_percent_);
  SetValue(config_json, &thresholds_kick_speed_);
  SetValue(config_json, &thresholds_follow_through_);
}

void PrimaryAttacker::Init() {
  intercept_solution_.isInitialized = false;
  catch_solution_.isInitialized = false;
}

void PrimaryAttacker::Reset() {
  state_ = start_;
  is_complete_ = false;
  kick_count_ = 0;
  aim_count_ = 0;
  last_target_score_ = 10.0;
  last_target_robot_ = 42;
  last_target_position_ = {0, 0};
  intercept_solution_.isInitialized = false;
  catch_solution_.isInitialized = false;
  // Reset the receiver when this tactic resets, otherwise the states
  // can hang between calls.
  Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::KICK].get();
  controller->Reset();
  controller = (*tactic_list_)[TacticIndex::THREE_KICK].get();
  controller->Reset();
}

void PrimaryAttacker::SetGoal(const Pose2Df& pose) {}

bool PrimaryAttacker::IsComplete() { return is_complete_; }

bool PrimaryAttacker::CouldBallScore(logger::Logger* the_logger,
                                     const float target_angle,
                                     const Vector2f current_ball_pose,
                                     const Vector2f current_ball_velocity,
                                     const Pose2Df current_robot_pose,
                                     const Pose2Df current_robot_velocity,
                                     const bool is_currenlty_kicking,
                                     const bool has_timed_out,
                                     const bool debug) {
  bool speed_check = false;

  float squared_dist;
  Vector2f intersect;

  bool passes_goal;

  passes_goal = RayIntersect(current_ball_pose,
                             current_ball_velocity,
                             kTheirGoalL,
                             kTheirGoalR,
                             &squared_dist,
                             &intersect);

  bool proximity;

  zone::FieldZone zone(zone::THEIR_HALF);
  proximity = zone.IsInZone(current_ball_pose);

  if (proximity && passes_goal) {
    speed_check = current_ball_velocity.norm() > thresholds_could_score_speed_;
  }
  return speed_check && passes_goal && proximity;
}

bool PrimaryAttacker::ShouldCatch(logger::Logger* the_logger,
                                  const float target_angle,
                                  const Vector2f current_ball_pose,
                                  const Vector2f current_ball_velocity,
                                  const Pose2Df current_robot_pose,
                                  const Pose2Df current_robot_velocity,
                                  const bool is_currenlty_kicking,
                                  const bool has_timed_out,
                                  const bool debug) {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrintPush("ShouldCatch");

  // SET POTENTIAL TRANSITION
  potential_state_ = "Receive";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  const bool speed_check =
      fabs(current_ball_velocity.norm()) > thresholds_lower_catch_velocity_;

  // Draw a line through the attacker perpendicular to the ball velocity
  const Vector2f robot_translation = current_robot_pose.translation;
  const Vector2f ball_perpendicular = Perp(current_ball_velocity);
  const float kMaxCatchRadius = kFieldLength;
  const Vector2f line_start =
      robot_translation + kMaxCatchRadius * ball_perpendicular;
  const Vector2f line_end =
      robot_translation - kMaxCatchRadius * ball_perpendicular;
  Vector2f intersect_point(0, 0);
  float intersect_distance = 0;
  const bool intersects = RayIntersect(current_ball_pose,
                                       current_ball_velocity,
                                       line_start,
                                       line_end,
                                       &intersect_distance,
                                       &intersect_point);
  // Check if the ball could intersect with the robot at all.
  if (intersects) {
    robot_logger->LogPrint("Intersects Attacker Line");
    const float attacker_dist =
        EuclideanDistance(intersect_point, robot_translation);
    const bool towards_robot = attacker_dist < thresholds_catch_radius_;

    const float catch_time =
        offense::GetNtocTime(current_robot_pose,
                             current_robot_velocity.translation,
                             intersect_point);

    const float ball_time = offense::GetBallTravelTime(
        current_ball_velocity.norm(), current_ball_pose, intersect_point);
    robot_logger->AddCircle(intersect_point, kBallRadius, 1.0, 0, 0, 1.0);
    bool in_time =
        (catch_time - ball_time) < 2 || attacker_dist < 2 * kRobotRadius;
        if (!in_time) {
          robot_logger->LogPrint("Time Difference %f", catch_time -
          ball_time);
        }
    // This disables the time check
    in_time = true;
    const bool could_score_check = !CouldBallScore(the_logger,
                                                   target_angle,
                                                   current_ball_pose,
                                                   current_ball_velocity,
                                                   current_robot_pose,
                                                   current_robot_velocity,
                                                   false,
                                                   false,
                                                   true);
    if (!speed_check) {
      robot_logger->LogPrint("Ball too slow");
    }
    if (!could_score_check) {
      robot_logger->LogPrint("Ball Could score");
    }
    if (!towards_robot) {
      robot_logger->LogPrint("Too far dist: = %f", attacker_dist);
    }
    robot_logger->Pop();
    const bool should_catch =
        speed_check && could_score_check && towards_robot && in_time;
    SetTransition(should_catch);
    return should_catch;
  } else {
    robot_logger->LogPrint("Doesn't Intersect");
  }
  SetTransition(false);
  return false;
}

bool PrimaryAttacker::ShouldKick(logger::Logger* logger,
                                 const float target_angle,
                                 const Vector2f current_ball_pose,
                                 const Vector2f current_ball_velocity,
                                 const Pose2Df current_robot_pose,
                                 const Pose2Df current_robot_velocity,
                                 const bool is_currenlty_kicking,
                                 const bool has_timed_out,
                                 const bool debug) {
  const bool kDebug = false;
  if (kDebug) {
    logger->LogPrintPush("ShouldKick");
  }
  const Eigen::Rotation2Df robot_to_world_rotation(current_robot_pose.angle);
  const Eigen::Rotation2Df world_to_target_rotation(-target_angle);
  const Eigen::Rotation2Df robot_to_target_rotation =
      world_to_target_rotation * robot_to_world_rotation;

  const Vector2f current_velocity_world =
      robot_to_world_rotation * current_robot_velocity.translation;

  const Vector2f robot_prime_vel =
      robot_to_target_rotation * current_robot_velocity.translation;

  const Vector2f robot_to_ball_displace =
      current_ball_pose - current_robot_pose.translation;
  Vector2f desired_norm(cos(target_angle), sin(target_angle));
  Vector2f collision_point =
      current_robot_pose.translation + kInterceptionRadius * desired_norm;
  const float robot_heading = current_robot_pose.angle;
  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  float y_dist = y_dir.dot(robot_to_ball_displace);
  float robot_y_prime_vel = robot_prime_vel.y();
  float ball_y_prime_vel =
      (world_to_target_rotation * current_ball_velocity).y();
  float rel_y_prime_vel = robot_y_prime_vel - ball_y_prime_vel;

  const Vector2f relative_velocity_vector =
      current_velocity_world - current_ball_velocity;

  // The boolean logic of "should_kick"

  // SET POTENTIAL TRANSITION
  potential_state_ = "Kick";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  double angle_diff =
      RadToDeg(fabs(AngleDiff(target_angle, current_robot_pose.angle)));
  const bool is_at_angle = angle_diff < thresholds_angle_;

  double angular_vel = RadToDeg(fabs(current_robot_velocity.angle));
  const bool is_rotation_at_rest = angular_vel < thresholds_angular_vel_;

  double radial_dist = (collision_point - current_ball_pose).norm();
  const bool is_at_radial_dist = radial_dist < thresholds_distance_;

  const bool is_y_prime_at_relative_rest =
      fabs(rel_y_prime_vel) < thresholds_y_prime_vel_;

  double relative_vel = (relative_velocity_vector).norm();
  const bool is_at_relative_rest = relative_vel < thresholds_relative_vel_;

  const bool is_in_alignment = fabs(y_dist) < thresholds_align_;

  if (kDebug) {
    if (!is_at_angle) {
      logger->LogPrint("Not At Angle. Angle Diff: %f, Thresh: %f",
                       angle_diff,
                       static_cast<float>(thresholds_angle_));
    }
    if (!is_rotation_at_rest) {
      logger->LogPrint("Not Rotation At Rest. Angular Vel: %f, Thresh: %f",
                       angular_vel,
                       static_cast<float>(thresholds_angular_vel_));
    }
    if (!is_at_radial_dist) {
      logger->LogPrint("Not At Radial Dist. Distance %f, Thresh: %f",
                       radial_dist,
                       static_cast<float>(thresholds_distance_));
    }
    if (!is_y_prime_at_relative_rest) {
      logger->LogPrint("Not at Relative Rest Y. Y Rel. Vel: %f, Thresh: %f",
                       fabs(rel_y_prime_vel),
                       static_cast<float>(thresholds_y_prime_vel_));
    }
    if (!is_at_relative_rest) {
      logger->LogPrint("Not at Relative Rest. Relative Vel: %f, Thresh: %f",
                       relative_vel,
                       static_cast<float>(thresholds_relative_vel_));
    }
    if (!is_in_alignment) {
      logger->LogPrint("Not in alignment. Y dist %f, Thresh: %f",
                       y_dist,
                       static_cast<float>(thresholds_align_));
    }
  }

  const bool should_kick =
      (is_at_angle && is_at_radial_dist && is_at_relative_rest &&
       is_in_alignment && is_rotation_at_rest && is_y_prime_at_relative_rest) ||
      (!has_timed_out && is_currenlty_kicking);

  // Don't kick the ball if it is in their defense area.
  obstacle::ObstacleFlag flags = obstacle::ObstacleFlag::GetDefenseAreas();
  for (auto obstacle : flags) {
    if (obstacle->PointCollision(current_ball_pose, kBallRadius)) {
      SetTransition(false);
      logger->Pop();
      return false;
    }
  }

  // SET SHOULD TRANSITION
  SetTransition(should_kick);
  if (kDebug) {
    logger->Pop();
  }
  return should_kick;
}

bool PrimaryAttacker::DoesPathCrossBallRay() {
  float dist;
  Vector2f point;
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  Vector2f desired_norm(cos(target_angle_), sin(target_angle_));
  desired_norm = -desired_norm;
  const Vector2f desired_point =
      robot_interception_point_ +
      ((kRobotRadius + kInterceptionRadius) * desired_norm);

  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  logger->AddCircle(desired_point, kRobotRadius, 1.0, 1.0, 0.0, 1.0);
  return RayIntersect(ball_pose,
                      ball_vel.normalized(),
                      current_pose.translation,
                      desired_point,
                      &dist,
                      &point);
}

bool PrimaryAttacker::ShouldIntercept() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  bool is_safe_path = !DoesPathCrossBallRay();

  zone::FieldZone field_zone(zone::FULL_FIELD);
  bool is_valid_intercept = field_zone.IsInZone(
      robot_interception_point_.cast<float>(), kDefaultSafetyMargin);

  bool is_legal_intercept = true;

  // Don't kick the ball if it is in their defense area.
  obstacle::ObstacleFlag flags = obstacle::ObstacleFlag::GetDefenseAreas();
  for (auto obstacle : flags) {
    if (obstacle->LineCollision(current_pose.translation,
                                robot_interception_point_,
                                kDefaultSafetyMargin)) {
      is_legal_intercept = false;
    }
  }

  ObstacleFlag obstacles = ObstacleFlag::GetAllExceptTeam(
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_);
  obstacles = obstacles & ~ObstacleFlag::GetBall();
  SafetyMargin safety_margin;

  bool is_collision_free = CollisionFreePath(obstacles,
                                             safety_margin,
                                             current_pose.translation,
                                             robot_interception_point_);

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrintPush("ShouldIntercept");
  if (!is_valid_intercept) {
    robot_logger->LogPrint("Intercept point not in field.");
  }
  if (!is_legal_intercept) {
    robot_logger->LogPrint("Intercept point in defense area. ");
  }
  if (!is_collision_free) {
    robot_logger->LogPrint("Intercept path not collision free.");
  }
  if (!is_safe_path) {
    robot_logger->LogPrint("Intercept path crosses ball trajectory.");
  }
  if (is_valid_intercept && is_legal_intercept && is_collision_free &&
      is_safe_path) {
    // SET POTENTIAL TRANSITION
    potential_state_ = "Intercept";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;
    const float ball_speed = ball_interception_vel_.norm();
    const bool ball_speed_check = ball_speed > thresholds_ball_velocity_;
    SetTransition(ball_speed_check);
    return ball_speed_check && is_valid_intercept && is_legal_intercept &&
           is_collision_free && is_safe_path;
  }
  robot_logger->Pop();
  return false;
}

bool PrimaryAttacker::ShouldGoToBall() {
  // SET POTENTIAL TRANSITION
  potential_state_ = "Start";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  bool speed_check = ball_vel.norm() < thresholds_ball_velocity_;
  // Also if the opponent possesses the ball....
  SetTransition(speed_check || soccer_state_->BallTheirPossession());
  return speed_check || soccer_state_->BallTheirPossession();
}

bool PrimaryAttacker::ShouldSTOXPivot(
    const Vector2f current_ball_pose,
    const Vector2f current_ball_velocity,
    const Pose2Df current_robot_pose,
    const Pose2Df current_robot_velocity) const {
  const bool ball_slow_enough = (current_ball_velocity.squaredNorm() < Sq(80));
  const bool ball_close_enough =
      ((current_ball_pose - current_robot_pose.translation).squaredNorm() <
       Sq(300));
  return (ball_slow_enough && ball_close_enough && false);
}

bool PrimaryAttacker::ShouldNavigateToCatch() {
  return false;
  //   logger::Logger* robot_logger =
  //       soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  //   const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  //   const Vector2f ball_pose = world_state_.GetBallPosition().position;
  //   const Pose2Df current_pose =
  //       world_state_.GetOurRobotPosition(our_robot_index_).position;
  //   const Pose2Df current_vel =
  //       world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  //
  //   // if ball moving towards our goal
  //   bool direction_check;
  //   if (kFieldSetup == LAB) {
  //     direction_check = false;
  //   } else {
  //     direction_check = Sign(ball_vel.x()) == -1;
  //   }
  //
  //   // if ball heading towards robot within angular margin
  //   Vector2f ball_dir = ball_vel.normalized();
  //   Vector2f displace_dir =
  //   (current_pose.translation - ball_pose).normalized();
  //   const float angle = acos(ball_dir.dot(displace_dir));
  //   bool angle_check = fabs(angle) < thresholds_toward_robot_;
  //   // if ball moving fast
  //   bool upper_speed_check = ball_vel.norm() > thresholds_ball_velocity_;
  //   bool could_score_check =
  //       !CouldBallScore(robot_logger, target_angle_, ball_pose, ball_vel,
  //                       current_pose, current_vel, false, false, false);
  //
  //   zone::FieldZone field_zone(zone::FULL_FIELD);
  //
  //   bool intercept_in_field =
  //       field_zone.IsInZone(robot_interception_point_, kDefaultSafetyMargin);
  //
  //   //   if (EuclideanDistance(ball_vel, current_pose.translation) <
  //   //       thresholds_catch_trajectory_) {
  //   //     return false;
  //   //   }
  //
  //   return (direction_check || angle_check || !intercept_in_field) &&
  //          upper_speed_check && could_score_check;
}

float PrimaryAttacker::GetCost() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  TacticIndex current_tactic =
      soccer_state_->GetRobotByOurRobotIndex(our_robot_index_).current_tactic_;
  logger->LogPrintPush("Primary Attacker Cost");
  const pose_2d::Pose2Df current_robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const pose_2d::Pose2Df current_robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const Vector2f current_ball_pose = world_state_.GetBallPosition().position;
  const Vector2f current_ball_velocity =
      world_state_.GetBallPosition().velocity;

  Eigen::Rotation2Df robot_to_world_rotation(current_robot_pose.angle);
  Vector2f current_velocity_world =
      robot_to_world_rotation * current_robot_velocity.translation;
  MotionModel model(kDefaultRobotAcceleration, kDefaultRobotVelocity);
  ControlSequence2D linear_control;
  static const float kNotcThreshold = 500;
  float cost = NTOC2D(current_robot_pose.translation - current_ball_pose,
                      current_velocity_world,
                      model,
                      &linear_control);

  // Get what state this robot would be in given the position of the ball.
  // Time will just be the best robot that can catch (imperfect unless a
  // robot cannot catch when it can't reach the intercept before the ball).
  if (ShouldCatch(logger,
                  target_angle_,
                  current_ball_pose,
                  current_ball_velocity,
                  current_robot_pose,
                  current_robot_velocity,
                  false,
                  false,
                  true)) {  // Time for ball to reach this robot.
    logger->LogPrint("Receive Cost");
    const Vector2f ball_to_robot_displace =
        current_robot_pose.translation - current_ball_pose;
    const float friction_decel = kBallAcceleration;
    float ball_other_soln = -1;
    const int ball_num_solns = SolveQuadratic(-0.5f * friction_decel,
                                              current_ball_velocity.norm(),
                                              -ball_to_robot_displace.norm(),
                                              &ball_other_soln,
                                              &cost);
    if (ball_num_solns == 0) {
      cost = 9999;
    } else if (ball_other_soln > 0) {
      cost = ball_other_soln;
    }
    class ::Receiver* controller = static_cast<class ::Receiver*>(
        (*tactic_list_)[TacticIndex::RECEIVER].get());
    if (controller->BadTiming()) {
      logger->LogPrint("Bad Timing Cost");
      cost = cost * 2;
    }
    // time will just be the closest robot to the ball
  } else if (current_ball_velocity.norm() < kNotcThreshold ||
             soccer_state_->BallTheirPossession()) {
    // cost should be ntoc time
    logger->LogPrint("Navigate Cost");
    //     cost = cost;
  } else {  // cost will be scaled based on its travel
    // direction with respect to this robot.
    logger->LogPrint("Angle Modified Cost");
    const Vector2f ball_observed = world_state_.GetBallPosition().observed_pose;
    Vector2f ball_to_bot = current_robot_pose.translation - ball_observed;
    ball_to_bot.normalize();
    float angle_from_robot = acos(ball_to_bot.dot(current_ball_velocity) /
                                  current_ball_velocity.norm());
    angle_from_robot = AngleMod(angle_from_robot);
    const float scaling = (1 + .5 * (1 - cos(angle_from_robot)));
    cost = scaling * cost;
  }
  // Dealing with the privileged states of the primary attacker.
  // Short circuit the cost
  if (current_tactic == PRIMARY_ATTACKER) {
    if (state_ == post_kick_) {
      cost = 9999;
    } else if (state_ == kick_) {
      return 0;
    }
    if (state_ == receive_) {
      Deflection* deflection = static_cast<Deflection*>(
          (*tactic_list_)[TacticIndex::DEFLECTION].get());
      if (deflection->IsKick()) {
        return 0;
      }
    }
  }

  if (shared_state_->IsPass() && shared_state_->GetPassShot() &&
      shared_state_->GetPassTarget() == our_robot_index_) {
    // We highly favor the pass receiving robot, but
    // this will occur for a small number of timesteps.
    logger->LogPrint("Reducing Cost For Pass");
    cost = cost * kPassReduction;
  }
  logger->Pop();
  return cost;
}

const Vector2f PrimaryAttacker::GetNextInterceptPoint() {
  Vector2f intercept_point = world_state_.GetBallPosition().position;
  if (state_ == intercept_ || state_ == navigate_to_intercept_) {
    intercept_point = ball_intercept_point_;
  }
  if (state_ == receive_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
    Pose2Df intercept = controller->GetGoal();
    intercept_point = intercept.translation;
  }
  return intercept_point;
}

const Pose2Df PrimaryAttacker::GetGoal() {
  const Vector2f intercept_point = GetNextInterceptPoint();
  return {0, intercept_point};
}

void PrimaryAttacker::GetSolution(Vector2f current_robot_trans,
                                  const Vector2f& current_velocity_world,
                                  const Vector2f& current_ball_pose,
                                  const Vector2f& current_ball_velocity) {
  InterceptionController* controller = static_cast<InterceptionController*>(
      (*tactic_list_)[TacticIndex::BALL_INTERCEPTION].get());
  controller->match_velocity_ = true;
  // find the solution for catch
  controller->SetSolution(catch_solution_);
  controller->SetOffset(Vector2f(0, 0));
  controller->RunDontCommand();
  robot_catch_point_ = controller->final_robot_pos_.cast<float>();
  robot_catch_vel_ = controller->final_ball_vel_.cast<float>();
  ball_catch_point_ = controller->final_ball_pos_.cast<float>();
  ball_catch_vel_ = controller->final_ball_vel_.cast<float>();
  catch_solution_ = controller->GetSolution();

  // find the solution for intercept
  controller->SetSolution(intercept_solution_);

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Aim Count: %d", aim_count_);
  if (aim_count_ < kMaxAimIterations) {
    OurRobotIndex target_robot = 0;
    Vector2f target_position = {0, 0};
    float new_target_angle;
    float last_target_angle = target_angle_;
    float score = GetTargetEvaluated(ball_intercept_point_,
                                     world_state_,
                                     soccer_state_,
                                     our_robot_index_,
                                     pass_only_,
                                     &chip_,
                                     &chip_distance_,
                                     &target_position,
                                     &new_target_angle,
                                     &target_robot,
                                     &last_target_);
    const Vector2f ball_displacement = current_robot_trans - current_ball_pose;
    if ((last_target_score_ - score) > kPassChangeThreshold_) {
      target_angle_ = new_target_angle;
    }
    robot_logger->LogPrint("Target Robot %d", target_robot);
    robot_logger->LogPrint("Target Angle %f", RadToDeg(target_angle_));
    robot_logger->LogPrint("Last Target %f", RadToDeg(last_target_angle));
    if (fabs((last_target_angle - target_angle_)) > DegToRad(1.0)) {
      if (ball_displacement.norm() < kRotationRadius) {
        aim_count_++;
      }
    }
    last_target_robot_ = target_robot;
  }

  Vector2f desired_norm(cos(target_angle_), sin(target_angle_));
  controller->SetOffset(kInterceptionRadius * desired_norm);
  controller->RunDontCommand();
  robot_interception_point_ = controller->final_robot_pos_.cast<float>();
  robot_interception_vel_ = controller->final_ball_vel_.cast<float>();
  ball_intercept_point_ = controller->final_ball_pos_.cast<float>();
  ball_interception_vel_ = controller->final_ball_vel_.cast<float>();
  intercept_solution_ = controller->GetSolution();
}

void PrimaryAttacker::GetTarget(const Vector2f& source,
                                float* target_angle,
                                OurRobotIndex* target_robot) {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  OurRobotIndex receiving_robot = 42;
  vector<Vector2f> world_robot_positions;
  for (const auto& robot : world_state_.GetOurRobots()) {
    if (world_state_.GetOurRobotIndex(robot.ssl_vision_id) !=
        static_cast<int>(our_robot_index_)) {
      world_robot_positions.push_back(robot.position.translation);
    }
  }
  int i = 0;
  for (const auto& robot : world_state_.GetTheirRobots()) {
    ++i;
    world_robot_positions.push_back(robot.position.translation);
  }
  is_complete_ = false;
  should_pass_ = false;
  vector<AimOption> aim_options;
  CalculateAimOptions(source,
                      kTheirGoalL,
                      kTheirGoalR,
                      kBallRadius,
                      kRobotRadius,
                      world_robot_positions,
                      &aim_options);
  if (!aim_options.empty()) {
    int max_width_index = -1;
    float max_width = -1;
    int current_index = 0;

    for (const AimOption& option : aim_options) {
      if (option.angle_width > max_width) {
        max_width = option.angle_width;
        max_width_index = current_index;
      }
      current_index++;
    }
    *target_angle = aim_options[max_width_index].angle_center;
  } else {
    should_pass_ = true;
  }
  // If no good aim option for shooting on the goal, pass the ball.
  if (should_pass_) {
    zone::FieldZone field_zone(zone::FULL_FIELD);
    *target_angle = GetBestPassTarget(
        world_state_,
        soccer_state_,
        our_robot_index_,
        world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id,
        field_zone,
        &receiving_robot);
  }
  *target_robot = receiving_robot;
  // Drawing a line from the robot in the direction of its target.
  const Vector2f target_direction = Heading(*target_angle);
  const Vector2f target_end =
      current_pose.translation + (4000 * target_direction);
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->AddLine(
      current_pose.translation, target_end, 1.0, 0.0, 1.0, 1.0);
}

// TODO(jaholtz) Change start to 'GoToBall', or something like that.
void PrimaryAttacker::Start() {
  intercept_solution_.isInitialized = false;
  catch_solution_.isInitialized = false;
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_vel =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  OurRobotIndex target_robot = offense::kNoPassRobot;
  Vector2f target_position = {0, 0};
  if (aim_count_ < kMaxAimIterations) {
    robot_logger->LogPrint("Aim Count: %d", aim_count_);
    const float last_target_angle = target_angle_;
    GetTargetEvaluated(ball_pose,
                       world_state_,
                       soccer_state_,
                       our_robot_index_,
                       pass_only_,
                       &chip_,
                       &chip_distance_,
                       &target_position,
                       &target_angle_,
                       &target_robot,
                       &last_target_);
    const Vector2f ball_displacement = ball_pose - current_pose.translation;
    robot_logger->LogPrint("Target Robot %d", target_robot);
    robot_logger->LogPrint("Target Angle %f", RadToDeg(target_angle_));
    robot_logger->LogPrint("Last Target %f", RadToDeg(last_target_angle));
    last_target_robot_ = target_robot;
    if (target_robot != offense::kNoPassRobot &&
        ball_displacement.norm() < kSetPassDistance) {
      shared_state_->SetPass(target_robot, target_position);
    } else {
      shared_state_->ClearPass();
    }
  }
  // Set the movement goal as a position behind the ball with respect
  // to the target, facing the ball.
  Vector2f desired_norm(cos(target_angle_), sin(target_angle_));
  Pose2Df goal_pose;
  goal_pose.translation = ball_pose - kRotationRadius * desired_norm;
  goal_pose.translation = ProjectToSafety(goal_pose.translation,
                                          current_pose.translation,
                                          kAttackerFieldMargin,
                                          robot_logger);
  goal_pose.angle = target_angle_;

  NTOC_Controller* ntoc =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  ntoc->TurnOnAngleRelaxation();

  EightGridNavigation* controller = static_cast<EightGridNavigation*>(
      (*tactic_list_)[TacticIndex::EIGHT_GRID].get());
  const float kSpeedThresh = 250;
  // Run the controller with the calculated goal and margins.
  controller->SetObstacles(
      obstacle::ObstacleFlag::GetAllExceptTeam(
          world_state_, *soccer_state_, our_robot_index_, our_robot_index_) |
      obstacle::ObstacleFlag::GetMediumBall());
  // If we're moving slow enough we want to get closer to the ball.
  if (current_vel.translation.norm() < kSpeedThresh) {
    controller->SetObstacles(
        obstacle::ObstacleFlag::GetAllExceptTeam(
            world_state_, *soccer_state_, our_robot_index_, our_robot_index_) |
        obstacle::ObstacleFlag::GetBall());
    safety::DSS2::SetObstacleFlag(our_robot_index_,
                                  obstacle::ObstacleFlag::GetEmpty());
  }
  controller->SetGoal(goal_pose);
  controller->Run();
}

void PrimaryAttacker::Catch() {
  Tactic* controller = (*tactic_list_)[TacticIndex::CATCH].get();
  controller->Run();
}

void PrimaryAttacker::Intercept() {
  intercepting_ = true;
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  InterceptionController* controller = static_cast<InterceptionController*>(
      (*tactic_list_)[TacticIndex::BALL_INTERCEPTION].get());
  controller->match_velocity_ = true;
  OurRobotIndex target_robot = offense::kNoPassRobot;
  Vector2f target_position(0, 0);
  float new_target_angle;
  if (aim_count_ < kMaxAimIterations) {
    chip_ = false;
    float score = GetTargetEvaluated(ball_intercept_point_,
                                     world_state_,
                                     soccer_state_,
                                     our_robot_index_,
                                     pass_only_,
                                     &chip_,
                                     &chip_distance_,
                                     &target_position,
                                     &new_target_angle,
                                     &target_robot,
                                     &last_target_);
    const Vector2f interception_diff =
        last_robot_interception_point_ - robot_interception_point_;
    if ((last_target_score_ - score) > kPassChangeThreshold_ ||
        (interception_diff.norm() > kEpsilon) ||
        target_robot == offense::kNoPassRobot) {
      target_angle_ = new_target_angle;
      last_target_score_ = score;
      last_target_robot_ = target_robot;
      last_target_position_ = target_position;
    }
    last_target_robot_ = target_robot;
  }
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  target_angle_ = offense::GetKickAngle(ball_interception_vel_,
                                        robot_interception_point_,
                                        offense::kKickSpeed,
                                        robot_logger,
                                        Heading(target_angle_));
  last_robot_interception_point_ = robot_interception_point_;
  const Vector2f target_displacement =
      robot_interception_point_ - current_pose.translation;
  if (last_target_robot_ != offense::kNoPassRobot &&
      target_displacement.norm() < kSetPassDistance) {
    safety::DSS2::SetObstacleFlag(our_robot_index_,
                                  obstacle::ObstacleFlag::GetDefenseAreas());
    if (world_state_.GetBallPosition().velocity.norm() < 3000) {
      shared_state_->SetPass(last_target_robot_, last_target_position_);
    }
  } else {
    shared_state_->ClearPass();
  }
  controller->SetSolution(intercept_solution_);
  controller->SetAngle(target_angle_);
  controller->Command();
}

void PrimaryAttacker::NavigateCatch() {
  NavigateToCatch* controller = static_cast<NavigateToCatch*>(
      (*tactic_list_)[TacticIndex::NAVIGATE_TO_CATCH].get());
  controller->SetSolution(catch_solution_);
  controller->Run();
}

void PrimaryAttacker::NavigateIntercept() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  NavigateToIntercept* controller = static_cast<NavigateToIntercept*>(
      (*tactic_list_)[TacticIndex::NAVIGATE_TO_INTERCEPTION].get());
  OurRobotIndex target_robot = offense::kNoPassRobot;
  Vector2f target_position(0, 0);
  if (aim_count_ < kMaxAimIterations) {
    GetTargetEvaluated(ball_intercept_point_,
                       world_state_,
                       soccer_state_,
                       our_robot_index_,
                       pass_only_,
                       &chip_,
                       &chip_distance_,
                       &target_position,
                       &target_angle_,
                       &target_robot,
                       &last_target_);
    last_target_robot_ = target_robot;
    const Vector2f ball_displacement = ball_pose - current_pose.translation;
    if (target_robot != offense::kNoPassRobot &&
        ball_displacement.norm() < kSetPassDistance) {
      if (world_state_.GetBallPosition().velocity.norm() < 3000) {
        shared_state_->SetPass(target_robot, target_position);
      }
    } else {
      shared_state_->ClearPass();
    }
  }
  controller->SetSolution(intercept_solution_);
  controller->SetAngle(target_angle_);
  controller->Run();
}

void PrimaryAttacker::Kick() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  if (shared_state_->IsPass()) {
    robot_logger->LogPrint("Passing");
  } else {
    robot_logger->LogPrint("Shooting");
  }
  if (intercepting_) {
    class ::Kick* controller =
        static_cast<class ::Kick*>((*tactic_list_)[TacticIndex::KICK].get());
    controller->SetGoal({target_angle_, 0, 0});
    controller->SetPassOnly(shared_state_->IsPass());
    controller->Run();
  } else {
    class ::ThreeKick* controller = static_cast<class ::ThreeKick*>(
        (*tactic_list_)[TacticIndex::THREE_KICK].get());
    controller->SetGoal({target_angle_, 0, 0});
    controller->SetPassOnly(shared_state_->IsPass());
    controller->Run();
  }
}

void PrimaryAttacker::PostKick() { Reset(); }

void PrimaryAttacker::Receive() {
  Tactic* controller = (*tactic_list_)[TacticIndex::RECEIVER].get();
  controller->Run();
}

void PrimaryAttacker::STOXPivot() {
  intercept_solution_.isInitialized = false;
  catch_solution_.isInitialized = false;
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_vel =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  OurRobotIndex target_robot = offense::kNoPassRobot;
  Vector2f target_position(0, 0);
  if (aim_count_ < kMaxAimIterations) {
    GetTargetEvaluated(ball_pose,
                       world_state_,
                       soccer_state_,
                       our_robot_index_,
                       pass_only_,
                       &chip_,
                       &chip_distance_,
                       &target_position,
                       &target_angle_,
                       &target_robot,
                       &last_target_);
    NP_CHECK(target_robot < kMaxTeamRobots ||
             target_robot == offense::kNoPassRobot);
    last_target_robot_ = target_robot;
    const Vector2f ball_displacement = ball_pose - current_pose.translation;
    robot_logger->LogPrint("Target Robot %d", target_robot);
    if (target_robot != offense::kNoPassRobot &&
        ball_displacement.squaredNorm() < Sq(kSetPassDistance)) {
      shared_state_->SetPass(target_robot, target_position);
    } else {
      shared_state_->ClearPass();
    }
  }
  // Set the movement goal as a position behind the ball with respect
  // to the target, facing the ball.
  Vector2f desired_norm(cos(target_angle_), sin(target_angle_));
  Pose2Df goal_pose;
  goal_pose.translation = ball_pose - kRotationRadius * desired_norm;
  goal_pose.translation = ProjectToSafety(goal_pose.translation,
                                          current_pose.translation,
                                          kAttackerFieldMargin,
                                          robot_logger);
  goal_pose.angle = target_angle_;

  tactics::STOXPivot* controller = static_cast<tactics::STOXPivot*>(
      (*tactic_list_)[TacticIndex::STOX_PIVOT].get());

  controller->SetGoal(goal_pose);
  controller->Run();
}

void PrimaryAttacker::Transition() {
  // Setup
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  // If going slowly, only the defense areas are obstacles for the attacker.
  // Helps with dueling.
  if (current_velocity.translation.norm() < 250) {
    safety::DSS2::SetObstacleFlag(our_robot_index_,
                                  obstacle::ObstacleFlag::GetDefenseAreas());
  }

  Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);

  Vector2f current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  // Gets the TSOCS solution used for ball intercept
  GetSolution(
      current_pose.translation, current_velocity_world, ball_pose, ball_vel);
  Deflection* deflection =
      static_cast<Deflection*>((*tactic_list_)[TacticIndex::DEFLECTION].get());
  State last_state = state_;
  // Don't transition out of kick unless kick decides too.
  if (state_ != kick_ && !deflection->IsKick()) {
    const bool should_kick = ShouldKick(robot_logger,
                                        target_angle_,
                                        ball_pose,
                                        ball_vel,
                                        current_pose,
                                        current_velocity,
                                        false,
                                        false,
                                        true);
    const bool should_catch = ShouldCatch(robot_logger,
                                          target_angle_,
                                          ball_pose,
                                          ball_vel,
                                          current_pose,
                                          current_velocity,
                                          false,
                                          false,
                                          true);
    const bool should_stox_pivot =
        ShouldSTOXPivot(ball_pose, ball_vel, current_pose, current_velocity);
    // Decide whether to pass or receive (or keep waiting)
    if (should_kick) {
      state_ = kick_;
      kick_solution_.isInitialized = false;
    } else if (should_stox_pivot) {
      state_ = stox_pivot_;
    } else if (ShouldGoToBall()) {
      state_ = start_;
    } else if (should_catch) {
      state_ = receive_;
    } else if (ShouldIntercept()) {
      if (state_ != intercept_) {
        intercept_solution_.isInitialized = false;
      }
      state_ = intercept_;
    } else {
      state_ = intercept_;
    }
  }

  // Special handling to break out of the receive state.
  if (state_ == receive_) {
    class ::Receiver* controller = static_cast<class ::Receiver*>(
        (*tactic_list_)[TacticIndex::RECEIVER].get());
    if (controller->IsComplete()) {
      Reset();
      Transition();
    } else if (controller->BadTiming()) {
      state_ = intercept_;
    }
  }
  // Check the controller completion conditions
  if (state_ == kick_) {
    Tactic* controller = (*tactic_list_)[TacticIndex::THREE_KICK].get();
    if (controller->IsComplete()) {
      state_ = post_kick_;
      is_complete_ = true;
    }
    controller = (*tactic_list_)[TacticIndex::KICK].get();
    if (controller->IsComplete()) {
      state_ = post_kick_;
      is_complete_ = true;
    }
  }
  // Used for knowing when to trigger intercept kick
  if (state_ != intercept_ && state_ != kick_) {
    intercepting_ = false;
  }
  if (state_ != last_state) {
    aim_count_ = 0;
  }
}

}  // namespace tactics
