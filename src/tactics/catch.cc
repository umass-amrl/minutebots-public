// Copyright 2017-2018 dbalaban@cs.umass.edu
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

#include "tactics/catch.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "safety/dss2.h"
#include "evaluators/offense_evaluation.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using geometry::Perp;
using math_util::AngleMod;
using geometry::SafeVectorNorm;
using state::SharedRobotState;
using state::WorldState;
using std::atan2;
using std::cos;
using std::endl;
using std::map;
using std::max;
using std::min;
using std::sin;
using std::abs;
using std::vector;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;
using ntoc::ControlPhase1D;
using ntoc::ControlSequence1D;
using ntoc::ControlPhase2D;
using motion::MotionModel;
using math_util::Sign;

namespace tactics {

Catch::Catch(const WorldState& world_state,
             TacticArray* tactic_list, SharedState* shared_state,
             OurRobotIndex our_robot_index, state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
             bad_time_(false),
             desired_accel_world_frame_(0, 0) {}

Catch::~Catch() {}

const Pose2Df Catch::GetGoal() {
  return world_state_.GetOurRobotPosition(our_robot_index_).position;
}

bool Catch::BadTiming() {
  return false;
}

void Catch::Run() {
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  the_logger->LogPrint("Running Catch");
  the_logger->Push();
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetDefenseAreas());

  const Pose2Df current_robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  const Pose2Df current_robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  const Vector2f current_ball_pose = world_state_.GetBallPosition().position;
  const Vector2f current_ball_velocity =
      world_state_.GetBallPosition().velocity;

  const Vector2f ball_dir = current_ball_velocity.normalized();
  const float ball_dir_angle = AngleMod(atan2(ball_dir.y(), ball_dir.x()));
  const float robot_heading = current_robot_pose.angle;

  Eigen::Rotation2Df robot_to_world_rotation(robot_heading);
  Eigen::Rotation2Df world_to_robot_rotation(-robot_heading);
  Eigen::Rotation2Df world_to_ball_dir_rotation(-ball_dir_angle);
  Eigen::Rotation2Df ball_dir_to_world_rotation(ball_dir_angle);
  Eigen::Rotation2Df robot_to_ball_dir_rotation =
      world_to_ball_dir_rotation * robot_to_world_rotation;

  Vector2f robot_ball_displace =
      current_robot_pose.translation - current_ball_pose;
  Vector2f displacement_ball_frame =
      world_to_ball_dir_rotation * robot_ball_displace;

  MotionModel angular_motion_model(kMaxRobotRotAccel, kMaxRobotRotVel);
  ControlSequence1D rotational_control;
  const float goal_angle = AngleMod(ball_dir_angle + M_PI);
  const double delta_angle = AngleDiff(goal_angle, current_robot_pose.angle);
  if (kDebug_) {
    the_logger->LogPrint("Target angle: %f", RadToDeg(goal_angle));
    the_logger->LogPrint("Current angdisplacement_ball_framele: %f",
                         RadToDeg(current_robot_pose.angle));
  }
  TimeOptimalControlAnyFinal1D(0.0, current_robot_velocity.angle, delta_angle,
                               0.0, 0.0, angular_motion_model,
                               &rotational_control);
  const float accel_angle = GetAverageAccel(rotational_control,
                                            kTransmitPeriodSeconds);

  float x_dist = displacement_ball_frame.x();
  float y_dist = displacement_ball_frame.y();
  Vector2f robot_velocity_ball_frame =
      robot_to_ball_dir_rotation * current_robot_velocity.translation;
  float robot_x_vel = robot_velocity_ball_frame.x();
  float robot_y_vel = robot_velocity_ball_frame.y();
  float ball_x_vel = current_ball_velocity.norm();
  float ball_y_vel = 0.0;

  if (kDebug_) {
    the_logger->LogPrint("Robot Distance to Ball (ball frame): (%f, %f)",
                         x_dist, y_dist);
    the_logger->LogPrint("Robot Vel (ball frame): (%f, %f)", robot_x_vel,
                         robot_y_vel);
    the_logger->LogPrint("Ball Vel (ball frame): (%f, %f)", ball_x_vel,
                         ball_y_vel);
  }

  float x_accel;
  if (fabs(robot_x_vel) >
      kMaxRobotAcceleration * kTransmitPeriodSeconds / sqrt(2.0)) {
    x_accel = -(Sign(robot_x_vel)) * kMaxRobotAcceleration/sqrt(2.0);
  } else {
    // stop
    x_accel = -robot_x_vel/kTransmitPeriodSeconds;
  }

  MotionModel linear_motion_model(kMaxRobotAcceleration / sqrt(2.0),
                                  kMaxRobotVelocity / sqrt(2.0));
  the_logger->LogPrint("y_dist: %f", y_dist);
  ControlSequence1D y_control;
  const float control_time = TimeOptimalControlAnyFinal1D(y_dist,
                                                    robot_y_vel,
                                                    0.0,
                                                    ball_y_vel,
                                                    0.0,
                                                    linear_motion_model,
                                                    &y_control);
  const float ball_travel_time = offense::GetBallTravelTime(
    current_ball_velocity.norm(),
    current_ball_pose,
    current_robot_pose.translation);
  bad_time_ = false;
  const float kCatchThreshold = .01;
  if (ball_travel_time - control_time > kCatchThreshold) {
    bad_time_ = true;
  }
  float y_accel = GetAverageAccel(y_control, kTransmitPeriodSeconds);
  y_control.LogSequence(the_logger, true);

  Vector2f desired_accel_ball_frame(x_accel, y_accel);
  desired_accel_world_frame_ =
      ball_dir_to_world_rotation* desired_accel_ball_frame;

  SharedRobotState* command = shared_state_->GetSharedState(our_robot_index_);
  command->our_robot_index = our_robot_index_;
  command->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  command->acceleration_command.translation = desired_accel_world_frame_;
  command->acceleration_command.angle = accel_angle;
  the_logger->Pop();
}

void Catch::Reset() {}

void Catch::Init() {}

void Catch::SetGoal(const pose_2d::Pose2Df& pose) {}

bool Catch::IsAtTargetPosition() {
  return SafeVectorNorm(desired_accel_world_frame_) < kEpsilon;
}

}  // namespace tactics
