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
#include "tactics/SimpleDeflection.h"

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
#include "debugging/tactic_exception.h"

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

SimpleDeflection::SimpleDeflection(const string& machine_name,
                                   const WorldState& world_state,
                                   TacticArray* tactic_list,
                                   SharedState* shared_state,
                                   OurRobotIndex our_robot_index,
                                   state::SoccerState* soccer_state)
    : StateMachineTactic("SimpleDeflection", world_state, tactic_list,
                         shared_state, our_robot_index, soccer_state),
      setup_(std::bind(&SimpleDeflection::Setup, this), "Setup"),
      kick_(std::bind(&SimpleDeflection::Kick, this), "Kick"),
      p_time_threshold_(0.1, "time_threshold", this),
      p_setup_distance_(0.85, "setup_distance", this),
      p_desired_accel_t_(1.0, "accel_t", this),
      p_desired_accel_a_(1.0, "accel_a", this),
      p_kick_dampening_(1.0, "kick_damping", this),
      p_kick_speed_(6000.0, "kick_speed", this),
      p_x_damping_(1.0, "x_damping", this),
      p_y_damping_(1.0, "y_damping", this),
      target_angle_(0) {
  state_ = setup_;
  LoadConfigFile();
}

void SimpleDeflection::SetValue(nlohmann::json config_json,
                               RepairableParam* param) {
  string name = param->name_;
  auto it = config_json.find(name);
  if (it != config_json.end()) {
    param->SetValue(config_json[name].get<float>());
  }
}


void SimpleDeflection::LoadConfigFile() {
  std::ifstream json_file("src/configs/simple_deflect_config.json");
  nlohmann::json config_json;
  json_file >> config_json;
  SetValue(config_json, &p_time_threshold_);
  SetValue(config_json, &p_desired_accel_t_);
  SetValue(config_json, &p_desired_accel_a_);
  SetValue(config_json, &p_kick_speed_);
  SetValue(config_json, &p_x_damping_);
}

void SimpleDeflection::CalculateScore() {
  // Distance of ball from the goal center in mm
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  distance_score_ = (kTheirGoalCenter - ball_pose).norm();

  // Ball velocity angle with respect to target angle
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const float ball_angle = Angle(ball_vel);
  const Vector2f robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  Vector2f distance = kTheirGoalCenter - robot_pose;
  const float goal_angle = Angle(distance);
  angle_score_ = fabs(AngleDiff(ball_angle, goal_angle));
}

float SimpleDeflection::GetCost() {
  return 0;
}

const Pose2Df SimpleDeflection::GetGoal() {
  return {0, kTheirGoalCenter};
}

void SimpleDeflection::Init() {
return;
}

bool SimpleDeflection::IsComplete() {
  return false;
}

void SimpleDeflection::Reset() {
 state_ = setup_;
}

void SimpleDeflection::SetGoal(const Pose2Df& pose) {
return;
}

float SimpleDeflection::GetKickAngle() {
  const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;

  const Vector2f robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f target_vector = kTheirGoalCenter - robot_pose;
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  const Vector2f final_velocity;
  const Vector2f neg_ball = -ball_velocity;
  float target_angle = Angle(target_vector);

  float least_error = 999;
  float right = Angle(neg_ball);
  float left = target_angle;
  const bool ball_left = right > left;
  if (right > left) {
    right = target_angle;

    left = Angle(neg_ball);
  }
  float best_angle = (left + right) / 2;

  while (fabs(least_error) > kEpsilon && fabs((left - right)) > DegToRad(3.0)) {
    float angle = AngleMod(left + right) / 2;
    best_angle = angle;
    // Calculating the reflected velocity
    const Eigen::Rotation2D<float> ball_to_robot_rotation(-angle);
    const Vector2f ball_vel_robot_frame =
        ball_to_robot_rotation * ball_velocity;
    const float ball_parallel =
        static_cast<float>(p_y_damping_) * ball_vel_robot_frame.y();
    const float ball_perp =
      static_cast<float>(p_x_damping_) * ball_vel_robot_frame.x();
    const Vector2f reflection_velocity = {-ball_perp, ball_parallel};
    const Vector2f kick_velocity = {static_cast<float>(p_kick_speed_), 0};

    // Sum the components and rotate back to world frame
    Vector2f resultant_vector = reflection_velocity + kick_velocity;
    resultant_vector.normalize();
    resultant_vector = ball_to_robot_rotation.inverse() * resultant_vector;
    const float result_angle = Angle(resultant_vector);

    const float current_error = AngleMod(target_angle - result_angle);
    robot_logger->LogPrint("Left %f, Right %f", left, right);
    robot_logger->LogPrint("Result Angle: %f, Error, %f", result_angle, current_error);
    if (ball_left) {
      if (current_error > 0) {
        right = angle;
      } else {
        left = angle;
      }
    } else {
      if (current_error > 0) {
        right = angle;
      } else {
        left = angle;
      }
    }

  }
  robot_logger->LogPrint("Target Angle %f", best_angle);
  //   const Vector2f final_vector = Heading(best_angle);
  return best_angle;
}

void SimpleDeflection::Setup() {
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  // Move to the wait positions
  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  // Set the safety margin for avoiding other robots
  // Run the controller with the calculated goal and margins.
  NTOC_Controller* ntoc =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  ntoc->TurnOffAngleRelaxation();
  Pose2Df target_pose;
  target_pose.translation =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  target_angle_ = GetKickAngle();
  target_pose.angle = target_angle_;
  controller->SetGoal(target_pose);
  controller->Run();
}

void SimpleDeflection::Kick() {
  kick_count_ += 1;
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  const float robot_angle =
      world_state_.GetOurRobotPosition(our_robot_index_).position.angle;
  const float omega =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.angle;
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  const Vector2f desired_acceleration(
      static_cast<float>(p_desired_accel_t_) * kMaxRobotAcceleration,
                                      0.0f);

  state->acceleration_command.translation =
      Rotation2Df(robot_angle) * desired_acceleration;

  ControlSequence1D rotational_control;
  TimeOptimalControlAnyFinal1D(
      robot_angle,
      omega,
      target_angle_,
      0.0,
      0.0,
      static_cast<float>(p_desired_accel_a_) * kMaxRobotRotAccel,
      kMaxRobotRotVel,
      &rotational_control);

  float angular_accel = GetAccelToPreservePosition(rotational_control,
                                                   kTransmitPeriodSeconds,
                                                   omega);

  state->acceleration_command.angle = angular_accel;
  state->flat_kick_set = true;
  const Vector2f robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_to_robot_displace = robot_pose - ball_pose;
  ball_to_robot_displace.normalize();
  const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
  const float relative_ball_vel = ball_to_robot_displace.dot(ball_velocity);
  const Vector2f robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
  float x_velocity = robot_velocity.x() / 1000;
  x_velocity +=
      static_cast<float>(p_kick_dampening_) * (relative_ball_vel / 1000);
  const float kick_speed = (static_cast<float>(p_kick_speed_) / 1000) - x_velocity;
  state->flat_kick = kick_speed;
}

Vector2f SimpleDeflection::GetInterceptionPoint() {
  const Vector2f ball_pose =
      world_state_.GetBallPosition().position;
  const Vector2f ball_velocity =
      world_state_.GetBallPosition().velocity;
  const Vector2f robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
  const Vector2f ball_moved = ball_pose + ball_velocity;
  if (ball_velocity.norm() < kEpsilon) {
    return robot_pose;
  }
  return LineLineIntersection(kTheirGoalCenter,
                              robot_pose,
                              ball_moved,
                              ball_pose);
}

 float SimpleDeflection::GetRobotKickTime(float robot_speed,
                                          float current_distance) {
  // Assumes that the robot has finished setup and is behind the line
  if (robot_speed >= kMaxRobotVelocity) {
    return current_distance / kMaxRobotVelocity;
  }

  const float desired_accel =
      static_cast<float>(p_desired_accel_t_) * kMaxRobotAcceleration;
  float t_1 = (kMaxRobotVelocity - robot_speed) / desired_accel;
  float s_1 = robot_speed * t_1 + 0.5f * desired_accel * t_1 * t_1;
  if (s_1 > current_distance) {
    // Does not reach max vel
    float t_short;
    float t_other;
    int num_solns = SolveQuadratic(0.5f * desired_accel,
                                   robot_speed,
                                   -current_distance,
                                   &t_other,
                                   &t_short);
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

float SimpleDeflection::GetBallArrivalTime(float ball_speed,
                                           float current_distance) {
  // Uses simulation ball model
  float ball_arrival_time = 0;
  float ball_other_soln = 0;
  int ball_num_solns = 0;
  // s = ut + 0.5at^2
  ball_num_solns =
      SolveQuadratic(-0.5f * kBallAcceleration, ball_speed, -current_distance,
                      &ball_other_soln, &ball_arrival_time);
  if (ball_num_solns == 0) {
    ball_arrival_time = -1.0f;  // is there a better constant for this
  } else if (ball_other_soln > 0) {
    ball_arrival_time = ball_other_soln;
  }
  return ball_arrival_time;
}

void SimpleDeflection::WriteDataFile(const string& task_string) {
  std::ofstream data_file;
  const string filename = "deflection_data/" + task_string + ".csv";
  string csv_tasks = task_string;
  std::replace(csv_tasks.begin(), csv_tasks.end(), '_', ',');
  data_file.open(filename, std::ofstream::out | std::ofstream::app);
  data_file << csv_tasks << ",";
  data_file << static_cast<float>(p_time_threshold_) << ",";
  data_file << static_cast<float>(p_desired_accel_t_) << ",";
  data_file << static_cast<float>(p_desired_accel_a_) << ",";
  data_file << static_cast<float>(p_kick_speed_) << ",";
  data_file << static_cast<float>(p_x_damping_) << ",";
  data_file << distance_score_ << "," << angle_score_ << endl;

}

void SimpleDeflection::CheckSuccess() {
  const Vector2f ball_vel = world_state_.GetBallPosition().velocity;
  const Vector2f ball_pose = world_state_.GetBallPosition().position;

  if (state_ == kick_) {
   Vector2f intersect_point;
   float square_distance = 0;
   if (geometry::RayIntersect(ball_pose,
                ball_vel,
                kTheirGoalL,
                kTheirGoalR,
                &square_distance,
                &intersect_point)) {
     if (square_distance < 1000 * 1000) {
      distance_score_ = (kTheirGoalCenter - intersect_point).norm();
      throw TacticException("Deflection Success");
     }
   }
  }
}

void SimpleDeflection::CheckFailure() {
  static constexpr int kMaxIterations = 150;
  static int count = 0;
  if (count > kMaxIterations) {
    throw TacticException("Deflection Failure");
  }
  ++count;
}

void SimpleDeflection::Transition() {
  CalculateScore();
  CheckSuccess();
  CheckFailure();
  const Vector2f intercept_point = GetInterceptionPoint();
  if (state_ == setup_) {
    // Calculate ball time to arrive
    const Vector2f ball_velocity = world_state_.GetBallPosition().velocity;
    const Vector2f ball_pose = world_state_.GetBallPosition().position;
    const float ball_distance = (intercept_point - ball_pose).norm();
    const float ball_time =
        GetBallArrivalTime(ball_velocity.norm(), ball_distance);

    // Robot Time to Kick the Ball
    const Vector2f robot_velocity =
        world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
    const Vector2f robot_trans =
        world_state_.GetOurRobotPosition(our_robot_index_).position.translation;
    const float robot_distance = (intercept_point - robot_trans).norm();
    const float robot_speed = robot_velocity.norm();
    const float robot_time =
        GetRobotKickTime(robot_speed, robot_distance);

    // Is the time sufficiently small?
    const float time_diff = robot_time - ball_time;
    const bool reach_in_time = fabs(time_diff) < p_time_threshold_;

    if (reach_in_time) {
      state_ = kick_;
    }
  }
}

}