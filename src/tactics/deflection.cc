// Copyright 2018 tszkeiserena@umass.edu, jaholtz@cs.umass.edu
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
#include "tactics/deflection.h"

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
#include "tactics/ntoc_controller.h"
#include "tactics/three_kick.h"
#include "zone/zone.h"

#define _USE_MATH_DEFINES

STANDARD_USINGS;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;
using pose_2d::Pose2Df;
using state::WorldState;
using state::WorldRobot;
using state::SoccerRobot;
using state::SharedState;
using std::unique_ptr;
using tactics::TacticIndex;
using tactics::ThreeKick;
using geometry::Angle;
using geometry::IsBetween;
using geometry::LineLineIntersection;
using geometry::Heading;
using geometry::ProjectPointOntoLineSegment;
using geometry::ProjectPointOntoLine;
using math_util::SolveQuadratic;
using motion::MotionModel;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence1D;
using ntoc::ControlSequence2D;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using state::SharedRobotState;
using offense::GetTarget;
using offense::GetTargetEvaluated;
using offense::GetKickAngle;
using offense::GetFinalBallSpeed;

namespace tactics {

Deflection::Deflection(const string& machine_name,
                       const WorldState& world_state, TacticArray* tactic_list,
                       SharedState* shared_state, OurRobotIndex our_robot_index,
                       state::SoccerState* soccer_state)
    : StateMachineTactic("TestStateMachineDeflection", world_state, tactic_list,
                         shared_state, our_robot_index, soccer_state),
      start_(std::bind(&Deflection::Start, this), "Start"),
      setup_(std::bind(&Deflection::Setup, this), "Setup"),
      wait_(std::bind(&Deflection::Wait, this), "Wait"),
      kick_(std::bind(&Deflection::Kick, this), "Kick"),
      finish_(std::bind(&Deflection::Finish, this), "Finish"),
      ball_angle_threshold_(DegToRad(100.0), 0.0, 3.14, "angle", this),
      ball_velocity_threshold_(0, 0.0, 5000.0, "ball_velocity_threshold", this),
      dribber_dampening_factor_(0.1, 0.0, 1.0, "dribber_dampening", this),
      ball_velocity_scale_(0.5, 0.0, 1.0, "ball_velocity_scale", this),
      angle_precision_threshold_(DegToRad(3.0), 0.0, 3.14,
                                 "angle_precision", this),
      robot_wait_threshold_y_max_(7, 0.0, kFieldLength,
                                  "robot_wait_vertical", this),
      robot_wait_threshold_tolerance_(5.2, 0.0, kFieldLength,
                                      "robot_wait_tolerance", this),
      robot_wait_threshold_x_(15, 0.0, kFieldLength,
                              "robot_wait_horizontal", this),
      time_diff_threshold_(0.85 * 1000, 0.0, 10000, "time_diff", this),
      kicked_x_threshold_(99,  0.0, 100.0, "kicked_x", this),
      kicked_velocity_threshold_(2, 0.0, 5000.0, "kicked_velocity", this),
      thresholds_kick_timeout_(40, 0.0, 1000.0, "kick_timeout", this),
      thresholds_kick_percent_(80, 0.0, 100.0, "kick_percent", this),
      thresholds_kick_speed_(1000, 0.0, 5000.0, "kick_speed", this),
      thresholds_follow_through_(20, 0.0, 100.0, "kick_follow_through", this),
      thresholds_distance_(kRotationRadius, 0.0, kFieldLength,
                           "distance", this),
      thresholds_setup_distance_(1000, 0.0, kFieldLength,
                                 "setup_distance", this),
      thresholds_deflection_angle_(offense::kDeflectionAngle, 0.0, 3.14,
                                   "deflection_angle", this),
      interception_point_(0, 0),
      interception_point_actual_(0, 0),
      interception_angle_goal_(0),
      interception_angle_actual_(0),
      waiting_buffer_(1.5 * kBallRadius),
      // large_angle_offset_base_(200.0f),
      kick_count_(0),
      simulation_(true),
      colinear_tolerance_(0.001f),
      complete_(false),
      passing_(false),
      last_target_(offense::kNoPassRobot) {
  state_ = setup_;
}

void Deflection::Init() { state_ = setup_; }

void Deflection::Reset() {
  state_ = setup_;
  passing_ = false;
  complete_ = false;
  kick_count_ = 0;
}

void Deflection::SetGoal(const Pose2Df& pose) {}

const Pose2Df Deflection::GetGoal() {
  if (state_ == kick_ || state_ == wait_) {
    return world_state_.GetOurRobotPosition(our_robot_index_).position;
  } else if (state_ == setup_) {
    return {0, interception_point_actual_};
  }
  return world_state_.GetOurRobotPosition(our_robot_index_).position;
}

bool Deflection::IsKick() { return state_ == kick_; }

float Deflection::GetCost() { return 0; }

bool Deflection::IsComplete() { return complete_; }

bool Deflection::BadDeflection() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const Vector2f current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f target_vector = Heading(interception_angle_actual_);
  // Get the speed of the ball at intercept position
  const float final_ball_speed =
      GetFinalBallSpeed(ball_vel.norm(), ball_pose, current_pose);
  const Vector2f final_ball_vel = final_ball_speed * ball_vel.normalized();
  // Determine if we should set up for a deflection, or a catch.
  float deflection_angle = 0;
  if (final_ball_vel.norm() > 0) {
    deflection_angle =
        acos(target_vector.dot(-final_ball_vel) / (final_ball_vel.norm()));
  } else {
    deflection_angle = acos(target_vector.dot(-final_ball_vel));
  }
  deflection_angle = AngleMod(deflection_angle);
  deflection_angle = fabs(deflection_angle);
  // Deflect or Catch?
  logger->LogPrint("Bad Deflection, Deflection Angle: %f",
                   RadToDeg(deflection_angle));
  zone::FieldZone field_zone(zone::FULL_FIELD);
  const Vector2f distance_from_deflect = ball_pose - interception_point_actual_;
  // SET POTENTIAL TRANSITION
  potential_state_ = "CATCH";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  bool angle_matters =
      distance_from_deflect.norm() > thresholds_setup_distance_;
  const bool bad_angle = deflection_angle > thresholds_deflection_angle_;
  SetTransition(angle_matters && bad_angle);
  const bool deflect_out_zone =
      !(field_zone.IsInZone(interception_point_actual_));
  return (angle_matters && bad_angle) || deflect_out_zone;
}

void Deflection::Start() {
  // Start state does nothing except check start conditions
}

void Deflection::Setup() {
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  // SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  // Move to the wait positions
  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  // Set the safety margin for avoiding other robots
  // Run the controller with the calculated goal and margins.
  NTOC_Controller* ntoc =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  ntoc->TurnOffAngleRelaxation();
  Pose2Df target_pose;
  target_pose.translation = interception_point_actual_;
  target_pose.angle = interception_angle_actual_;
  controller->SetGoal(target_pose);
  controller->Run();
}

void Deflection::Wait() {
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  //   Tactic* controller = (*tactic_list_)[TacticIndex::STOPPED].get();
  kick_count_ = 0;
  //   controller->Run();
  // SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  // Move to the wait positions
  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  // Set the safety margin for avoiding other robots
  // Run the controller with the calculated goal and margins.
  Pose2Df target_pose;
  target_pose.translation = interception_point_actual_;
  target_pose.angle = interception_angle_actual_;
  controller->SetGoal(target_pose);
  controller->Run();
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  // Wait sets kick
  // (in case the ball hits us but we failed to enter kick somehow)
  if (shared_state_->IsPass()) {
    state->flat_kick = 1.5;
  } else {
    state->flat_kick = 5.0;
    shared_state_->ClearPass();
  }
}

void Deflection::Kick() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  float robot_angle =
      world_state_.GetOurRobotPosition(our_robot_index_).position.angle;
  float omega =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.angle;
  kick_count_ += 1;
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  Vector2f desired_acceleration(kMaxRobotAcceleration, 0.0f);

  state->acceleration_command.translation =
      Rotation2Df(robot_angle) * desired_acceleration;

  ControlSequence1D rotational_control;
  TimeOptimalControlAnyFinal1D(robot_angle, omega, interception_angle_actual_,
                               0.0, 0.0, kMaxRobotRotAccel, kMaxRobotRotVel,
                               &rotational_control);

  float angular_accel = GetAccelToPreservePosition(
      rotational_control, kTransmitPeriodSeconds, omega);

  state->acceleration_command.angle = angular_accel;

  state->flat_kick_set = true;
  const Vector2f robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_to_robot_displace = robot_pose - ball_pose;
  ball_to_robot_displace.normalize();
  const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  const float relative_ball_vel = ball_to_robot_displace.dot(ball_velocity);
  robot_logger->LogPrint("Relative Ball Vel: %f", relative_ball_vel);
  if (passing_) {
    const Vector2f robot_velocity =
        world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
    float x_velocity = robot_velocity.x() / 1000;
    x_velocity += (relative_ball_vel / 1000);
    const float desired_speed = offense::kPassSpeed / 1000;
    const float kick_speed = desired_speed - fabs(x_velocity);
    state->flat_kick = kick_speed;
    robot_logger->LogPrint("Desired Kick: %f", desired_speed);
    robot_logger->LogPrint("x_velocity: %f", x_velocity);
    robot_logger->LogPrint("kick_speed: %f", kick_speed);
    shared_state_->SetPassShot();
  } else {
    const Vector2f robot_velocity =
        world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
    float x_velocity = robot_velocity.x() / 1000;
    x_velocity += .2 * (relative_ball_vel / 1000);
    const float desired_speed = 6.5;
    const float kick_speed = desired_speed - x_velocity;
    robot_logger->LogPrint("Desired Kick: %f", desired_speed);
    robot_logger->LogPrint("x_velocity: %f", x_velocity);
    robot_logger->LogPrint("kick_speed: %f", kick_speed);
    state->flat_kick = kick_speed;
  }
}

void Deflection::Finish() {
  complete_ = true;
  Tactic* controller = (*tactic_list_)[TacticIndex::HALT].get();
  controller->Run();
  shared_state_->ClearPass();
}

float Deflection::GetRobotKickTime(float robot_speed, float current_distance) {
  // Assumes that the robot has finished setup and is behind the line
  if (robot_speed >= kMaxRobotVelocity) {
    return current_distance / kMaxRobotVelocity;
  }

  float t_1 = (kMaxRobotVelocity - robot_speed) / kMaxRobotAcceleration;
  float s_1 = robot_speed * t_1 + 0.5f * kMaxRobotAcceleration * t_1 * t_1;
  if (s_1 > current_distance) {
    // Does not reach max vel
    float t_short;
    float t_other;
    int num_solns = SolveQuadratic(0.5f * kMaxRobotAcceleration, robot_speed,
                                   -current_distance, &t_other, &t_short);
    if (num_solns == 0) {
      return -1.0f;
    } else {
      if (t_other > 0) {
        t_short = t_other;
      }
      return t_short;
    }
  } else {
    // Reaches max vel
    float t_2 = (current_distance - s_1) / kMaxRobotVelocity;
    return t_1 + t_2;
  }
}

float Deflection::GetBallArrivalTime(float ball_speed, float current_distance) {
  // Uses simulation ball model
  float ball_arrival_time = 0;
  float ball_other_soln = 0;
  int ball_num_solns = 0;
  // s = ut + 0.5at^2
  if (simulation_) {
    ball_num_solns =
        SolveQuadratic(-0.5f * kBallAcceleration, ball_speed, -current_distance,
                       &ball_other_soln, &ball_arrival_time);
  } else {
    ball_num_solns =
        SolveQuadratic(-0.5f * kBallAcceleration, ball_speed, -current_distance,
                       &ball_other_soln, &ball_arrival_time);
  }
  if (ball_num_solns == 0) {
    ball_arrival_time = -1.0f;  // is there a better constant for this
  } else if (ball_other_soln > 0) {
    ball_arrival_time = ball_other_soln;
  }
  return ball_arrival_time;
}

Vector2f Deflection::ShiftInterceptionPoint(Vector2f current_point,
                                            Vector2f dir_vector, float offset) {
  return current_point + (offset * dir_vector);
}

Vector2f Deflection::GetInterceptionPoint(Vector2f robot_current,
                                          Vector2f robot_goal,
                                          Vector2f ball_current,
                                          Vector2f ball_velocity, float tol) {
  Vector2f robot_vec = robot_goal - robot_current;
  robot_vec.normalize();
  ball_velocity.normalize();
  Vector2f ball_projected;
  ProjectPointOntoLine(ball_current, robot_current, robot_goal,
                       &ball_projected);
  Vector2f ball_projected_robot_dir = robot_goal - ball_projected;
  ball_projected_robot_dir.normalize();
  float ball_robot_dot_normalized =
      1.0f - ball_projected_robot_dir.dot(robot_vec);
  float velocity_robot_dot_normalized = 1.0f + ball_velocity.dot(robot_vec);
  if (ball_robot_dot_normalized < tol && velocity_robot_dot_normalized < tol) {
    return ShiftInterceptionPoint(robot_current, ball_projected_robot_dir,
                                  kWaitingDistance);
  } else {
    Vector2f ball_moved = ball_current + ball_velocity;
    return LineLineIntersection(robot_current, robot_goal, ball_current,
                                ball_moved);
  }
}

bool Deflection::BadTiming() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  if (state_ != kick_) {
    const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
    const Vector2f ball_pose = world_state_.GetBallPosition().position;
    const Pose2Df robot_pose =
        world_state_.GetOurRobotPosition(our_robot_index_).position;
    const Vector2f robot_vel =
        world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
    const float ball_travel_time = offense::GetBallTravelTime(
        ball_velocity.norm(), ball_pose, interception_point_actual_);
    float time_to_receive =
        offense::GetNtocTime(robot_pose, robot_vel, interception_point_actual_);
    const float kReceiveThreshold = 10.0;
    logger->LogPrint("Travel Time Difference %f",
                     ball_travel_time - time_to_receive);
    if (ball_travel_time - time_to_receive < kReceiveThreshold) {
      return true;
    }
  }
  return false;
}

bool Deflection::SettingUp() { return state_ == setup_; }

void Deflection::Transition() {
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  // Gets robot position and velocity
  const Vector2f robot_position =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const float robot_angle =
      world_state_.GetOurRobotPosition(our_robot_index_).position.angle;
  const Vector2f robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;

  // Gets ball position, vel, and projected position for calculation
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  // const Vector2f ball_moved = ball_pose + ball_velocity;

  const float ball_speed = ball_velocity.norm();

  OurRobotIndex target_robot = offense::kNoPassRobot;
  Vector2f target_position;
  GetTargetEvaluated(robot_position, world_state_, soccer_state_,
                     our_robot_index_, false, &target_position,
                     &interception_angle_goal_, &target_robot, &last_target_);
  float kick_speed = offense::kKickSpeed;
  if (target_robot != offense::kNoPassRobot) {
    kick_speed = offense::kPassSpeed;
  }
  const float final_ball_speed =
      GetFinalBallSpeed(ball_speed, ball_pose, robot_position);
  const Vector2f final_ball_vel = final_ball_speed * ball_velocity.normalized();
  interception_angle_goal_ =
      GetKickAngle(final_ball_vel, robot_position, kick_speed, logger,
                   Heading(interception_angle_goal_));
  // Don't change your angle in kick or wait...
  if (state_ != kick_ && state_ != wait_) {
    if (target_robot != offense::kNoPassRobot) {
      passing_ = true;
      shared_state_->SetPass(target_robot, target_position);
    } else {
      passing_ = false;
      shared_state_->ClearPass();
    }
    interception_angle_actual_ = interception_angle_goal_;
  }
  const Vector2f to_goal = Heading(interception_angle_actual_);

  interception_point_ = GetInterceptionPoint(
      robot_position, robot_position + Heading(interception_angle_actual_),
      ball_pose, ball_velocity, colinear_tolerance_);
  Vector2f ball_distance_from_point = interception_point_ - ball_pose;

  // Set up basic deflection waiting position
  const Vector2f y_dir(-sin(interception_angle_actual_),
                       cos(interception_angle_actual_));
  const Vector2f x_dir = Heading(interception_angle_actual_);
  interception_point_ = ShiftInterceptionPoint(
      interception_point_, Heading(interception_angle_actual_), -kRobotRadius);
  interception_point_actual_ = interception_point_;
  interception_point_actual_ = ShiftInterceptionPoint(
      interception_point_, Heading(interception_angle_actual_),
      -waiting_buffer_);

  // Calculates projected time until ball reaches intersection of path
  float ball_arrival_time =
      GetBallArrivalTime(ball_speed, ball_distance_from_point.norm());
  float next_ball_time = ball_arrival_time - kTransmitPeriodSeconds;
  if (ball_arrival_time == -1.0f) {
    logger->LogPrint("Something wrong with ball time calculation");
  }

  // Modifies interception point for large angles
  // Calculates angle between ball vector and predicted deflected trajectory
  const float approaching_angle = std::acos(
      to_goal.dot(-ball_distance_from_point) / ball_distance_from_point.norm());

  // Calculates robot distances from interception points
  Vector2f displacement = interception_point_ - robot_position;
  Vector2f distance_from_waiting = interception_point_actual_ - robot_position;
  float current_distance = displacement.norm();

  // Difference in the time for the ball and robot to reach intersect point
  const float time_diff = ball_arrival_time;

  // distance calculation
  const float x_dist_from_intercept = x_dir.dot(distance_from_waiting);
  const float y_dist_from_intercept = y_dir.dot(distance_from_waiting);

  // Calculates whether things are being kicked around
  const Vector2f robot_dir = Heading(robot_angle);
  const float relative_ball_vel = robot_dir.dot(ball_velocity);
  const float relative_percent = (relative_ball_vel / ball_speed) * 100;

  // Print debug info
  logger->LogPrint("Time of ball arrival: %f", ball_arrival_time);
  logger->LogPrint("Next Ball Time: %f", next_ball_time);
  if (kDebug) {
    logger->LogPrint("Interception point: %f, %f", interception_point_.x(),
                     interception_point_.y());
    logger->LogPrint("Interception actual: %f, %f",
                     interception_point_actual_.x(),
                     interception_point_actual_.y());
    logger->LogPrint("Robot position: %f, %f", robot_position.x(),
                     robot_position.y());
    logger->LogPrint("Robot distance from interception point: %f",
                     current_distance);
    logger->LogPrint("Robot distance vector from waiting: %f, %f",
                     distance_from_waiting.x(), distance_from_waiting.y());
    logger->LogPrint("Ball position: %f, %f", ball_pose.x(), ball_pose.y());
    logger->LogPrint("Ball distance from interception point: %f",
                     ball_distance_from_point.norm());
    logger->LogPrint("Goal position: %f, %f", kTheirGoalCenter.x(),
                     kTheirGoalCenter.y());
    logger->LogPrint("Approaching angle: %f", approaching_angle);
    logger->LogPrint("x_dir vector: %f, %f", x_dir.x(), x_dir.y());
    logger->LogPrint("y_dir vector: %f, %f", y_dir.x(), y_dir.y());
    logger->LogPrint("x distance from intercept: %f", x_dist_from_intercept);
    logger->LogPrint("y distance from intercept: %f", y_dist_from_intercept);
    logger->LogPrint("Relative Percent %f", relative_percent);
    logger->LogPrint("Robot velocity: %f, %f", robot_velocity.x(),
                     robot_velocity.y());
    logger->LogPrint("Robot speed: %f", robot_velocity.norm());
    logger->LogPrint("Ball velocity: %f, %f", ball_velocity.x(),
                     ball_velocity.y());
    logger->LogPrint("Ball speed: %f", ball_velocity.norm());
  }
  logger->LogPrint("Target angle: %f", interception_angle_actual_);
  logger->LogPrint("Current angle: %f", robot_angle);

  // Shows interception point on viewer
  if (kDebug) {
    logger->AddCircle(interception_point_, 100, 1.0, 0.0, 1.0, 1.0);
    logger->AddCircle(interception_point_actual_, 100, 1.0, 1.0, 1.0, 1.0);
  }

  // Controls transitions between states
  if (state_ == setup_) {
    // Set potential transition
    potential_state_ = "Wait";
    // Add a block of clauses
    AddBlock(true);
    // Next clauses are ands
    and_clause_ = true;

    // Calculates difference between current robot angle and target
    const float angle_diff = AngleDiff(interception_angle_actual_, robot_angle);
    const bool at_angle = fabs(angle_diff) < angle_precision_threshold_;
    const bool at_waiting_distance_y =
        fabs(y_dist_from_intercept) < robot_wait_threshold_tolerance_;
    const bool at_waiting_distance_x_upper =
        fabs(x_dist_from_intercept) < robot_wait_threshold_x_;

    logger->LogPrint("At waiting distance y: %s",
                     at_waiting_distance_y ? "true" : "false");

    logger->LogPrint("At waiting distance x: %s",
                     at_waiting_distance_x_upper ? "true" : "false");

    logger->LogPrint("X Dist from Intercept: %f", x_dist_from_intercept);

    logger->LogPrint("Angle Offset: %f", RadToDeg(fabs(angle_diff)));

    // Scaling because SRTR likes whole numbers better than decimals
    const float scaled_time_diff = time_diff * 10000;
    const bool reach_in_time =
        scaled_time_diff < static_cast<float>(time_diff_threshold_) * .8;

    const bool at_waiting_distance =
        at_waiting_distance_x_upper && at_waiting_distance_y;

    SetTransition(at_angle && at_waiting_distance);
    if (at_angle && at_waiting_distance) {
      state_ = wait_;
    } else if (at_angle && at_waiting_distance_y && reach_in_time) {
      state_ = kick_;
    } else if (at_waiting_distance_x_upper && at_waiting_distance_y &&
               reach_in_time) {
      state_ = kick_;
    }
  } else if (state_ == wait_) {
    // Set potential transition
    potential_state_ = "Kick";
    // Add a block of clauses
    AddBlock(true);
    // Next clauses are ands
    and_clause_ = true;

    // Scaling because SRTR likes whole numbers better than decimals
    const float scaled_time_diff = time_diff * 10000;
    const float next_scaled_time_diff = next_ball_time * 10000;

    const bool reach_in_time = fabs(scaled_time_diff) < time_diff_threshold_;
    AddBlock(false);
    const bool reach_in_next_time =
        fabs(next_scaled_time_diff) < time_diff_threshold_;
    const float this_time_offset =
        fabs(scaled_time_diff - static_cast<float>(time_diff_threshold_));
    const float next_time_offset =
        fabs(next_scaled_time_diff - static_cast<float>(time_diff_threshold_));

    SetTransition(reach_in_time ||
                  (reach_in_next_time && this_time_offset < next_time_offset));
    if (reach_in_time ||
        (reach_in_next_time && this_time_offset < next_time_offset)) {
      state_ = kick_;
    }
  } else if (state_ == kick_) {
    bool kicked = false;
    // SET POTENTIAL TRANSITION
    potential_state_ = "POSTKICK";
    // ADD A BLOCK OF CLAUSES
    AddBlock(true);
    // SET AND FOR CLAUSES
    and_clause_ = true;
    const float ball_target_projection =
        ball_velocity.dot(Heading(interception_angle_actual_));
    const float ball_target_percent =
        1000 * (ball_target_projection / ball_velocity.norm());
    const Vector2f offset = ball_pose - robot_position;
    const bool timed_out = kick_count_ > thresholds_kick_timeout_;
    kicked = ball_target_percent > thresholds_kick_percent_ &&
             robot_position.norm() > thresholds_kick_speed_ &&
             kick_count_ > thresholds_follow_through_ &&
             offset.norm() > thresholds_distance_;
    SetTransition(timed_out);
    if (timed_out || kicked) {
      state_ = finish_;
    }
  } else if (state_ == finish_) {
    Reset();
  }
}

}  // namespace tactics
