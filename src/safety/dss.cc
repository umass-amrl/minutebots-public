// Copyright 2017 - 2019 kvedder@umass.edu
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

#include "safety/dss.h"

#include <algorithm>
#include <random>
#include <vector>

#include "constants/constants.h"
#include "logging/logger.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/rectangle_obstacle.h"
#include "safety/dss_helpers.h"
#include "state/shared_state.h"
#include "state/world_state.h"

STANDARD_USINGS;
using Eigen::Rotation2Df;
using pose_2d::Pose2Df;
using obstacle::CircleObstacle;
using obstacle::RectangleObstacle;
using state::SharedRobotState;
using state::SoccerState;
using state::WorldRobot;
using state::WorldState;
using state::PositionVelocityState;
using std::uniform_real_distribution;
using std::vector;
using math_util::SolveQuadratic;
using math_util::SolveCubic;
using math_util::Sq;
using math_util::Cube;
using math_util::Pow;
using geometry::GetNormalizedOrZero;
using logger::Logger;
using motion::MotionModel;

namespace safety {

DSS::DSS(const WorldState& world_state, SoccerState* soccer_state)
    : random(),
      world_state_(world_state),
      soccer_state_(soccer_state) {}

DSS::~DSS() {}

const bool IsInCollision(const float squared_distance, const float robot_radius,
                         const float safety_margin) {
  return Sq(2 * robot_radius + safety_margin) > squared_distance;
}

// Tests if the robots will collide during the control period.
bool IsCollidingDuringControlPeriod(
    Logger* current_robot_logger, const Vector2f& robot_1_position,
    const Vector2f& robot_1_current_velocity,
    const Vector2f& robot_1_command_velocity,
    const motion::MotionModel& robot_1_motion_model,
    const Vector2f& robot_2_position, const Vector2f& robot_2_current_velocity,
    const Vector2f& robot_2_command_velocity,
    const motion::MotionModel& robot_2_motion_model, const float robot_radius,
    const float control_period, const float safety_margin) {
  // Acceleration over the whole control period. This is a constant as per the
  // definition of a timeslice.
  const Vector2f robot_1_acceleration =
      CapAcceleration(robot_1_command_velocity - robot_1_current_velocity,
                      robot_1_motion_model);
  const Vector2f robot_2_acceleration =
      CapAcceleration(robot_2_command_velocity - robot_2_current_velocity,
                      robot_2_motion_model);

  const Vector2f relative_robot_position_start =
      robot_2_position - robot_1_position;
  const Vector2f relative_robot_current_velocity_start =
      robot_2_current_velocity - robot_1_current_velocity;
  const Vector2f relative_robot_acceleration_start =
      robot_2_acceleration - robot_1_acceleration;

  const float closest_point_time = ClosestDistanceTime(
      relative_robot_position_start, relative_robot_current_velocity_start,
      relative_robot_acceleration_start);

  if (closest_point_time >= 0 && closest_point_time <= control_period) {
    const float closest_distance_squared = DistanceSquared(
        relative_robot_position_start, relative_robot_current_velocity_start,
        relative_robot_acceleration_start, control_period);
    if (IsInCollision(closest_distance_squared, robot_radius, safety_margin)) {
      return true;
    }
  }
  const float closest_distance_start_squared = DistanceSquared(
      relative_robot_position_start, relative_robot_current_velocity_start,
      relative_robot_acceleration_start, 0.0f);
  const float closest_distance_end_squared = DistanceSquared(
      relative_robot_position_start, relative_robot_current_velocity_start,
      relative_robot_acceleration_start, control_period);
  return ((IsInCollision(closest_distance_end_squared, robot_radius,
                         safety_margin)) ||
          (IsInCollision(closest_distance_start_squared, robot_radius,
                         safety_margin)));
}

// Checks if the robots collide during the period where they are both slowing.
bool IsCollidingDuringSlowSlowPeriod(
    Logger* current_robot_logger, const Vector2f& robot_1_position,
    const Vector2f& robot_1_velocity, const Vector2f& robot_1_acceleration,
    const Vector2f& robot_2_position, const Vector2f& robot_2_velocity,
    const Vector2f& robot_2_acceleration, const float robot_radius,
    const float control_period, const float safety_margin) {
  const Vector2f relative_position = robot_1_position - robot_2_position;
  const Vector2f relative_velocity = robot_1_velocity - robot_2_velocity;
  const Vector2f relative_acceleration =
      robot_1_acceleration - robot_2_acceleration;

  const float closest_time = ClosestDistanceTime(
      relative_position, relative_velocity, relative_acceleration);

  if (closest_time >= 0 && closest_time <= control_period) {
    const float closest_distance_squared =
        DistanceSquared(relative_position, relative_velocity,
                        relative_acceleration, closest_time);
    if (IsInCollision(closest_distance_squared, robot_radius, safety_margin)) {
      return true;
    }
  }

  const float closest_distance_start = DistanceSquared(
      relative_position, relative_velocity, relative_acceleration, 0);
  const float closest_distance_end =
      DistanceSquared(relative_position, relative_velocity,
                      relative_acceleration, control_period);

  return (
      (IsInCollision(closest_distance_start, robot_radius, safety_margin)) ||
      (IsInCollision(closest_distance_end, robot_radius, safety_margin)));
}

bool IsCollidingDuringSlowStopPeriod(
    Logger* current_robot_logger, const Vector2f& moving_position,
    const Vector2f& moving_velocity, const Vector2f& moving_deceleration,
    const Vector2f& stopped_position, const float robot_radius,
    const float control_period, const float safety_margin) {
  const Vector2f relative_position = moving_position - stopped_position;
  const Vector2f relative_velocity = moving_velocity;
  const Vector2f relative_acceleration = moving_deceleration;

  const float closest_time = ClosestDistanceTime(
      relative_position, relative_velocity, relative_acceleration);

  const float min_safe_distance_squared = Sq(2 * robot_radius + safety_margin);

  if (closest_time >= 0 && closest_time <= control_period) {
    const float closest_distance_squared =
        DistanceSquared(relative_position, relative_velocity,
                        relative_acceleration, closest_time);
    if (closest_distance_squared < min_safe_distance_squared) {
      return true;
    }
  }

  const float closest_distance_start = DistanceSquared(
      relative_position, relative_velocity, relative_acceleration, 0);
  const float closest_distance_end =
      DistanceSquared(relative_position, relative_velocity,
                      relative_acceleration, control_period);
  return ((closest_distance_start < min_safe_distance_squared) ||
          (closest_distance_end < min_safe_distance_squared));
}

// Checks to see if the robots collide while both are slowing down.
bool IsCollidingDuringHaltingPeriod(
    Logger* current_robot_logger, const Vector2f& robot_1_position,
    const Vector2f& robot_1_current_velocity,
    const motion::MotionModel& robot_1_motion_model,
    const Vector2f& robot_2_position, const Vector2f& robot_2_current_velocity,
    const motion::MotionModel& robot_2_motion_model, const float robot_radius,
    const float safety_margin) {
  static const bool kDebug = false;
  const float robot_1_stop_time =
      CalculateTimeToRest(robot_1_current_velocity, robot_1_motion_model);
  const float robot_2_stop_time =
      CalculateTimeToRest(robot_2_current_velocity, robot_2_motion_model);

  const Vector2f robot_1_deceleration =
      GetNormalizedOrZero(robot_1_current_velocity) *
      -robot_1_motion_model.a_max;
  const Vector2f robot_2_deceleration =
      GetNormalizedOrZero(robot_2_current_velocity) *
      -robot_2_motion_model.a_max;

  const Vector2f robot_1_final_stop_delta_position = CalculateDeltaPosition(
      robot_1_current_velocity, robot_1_deceleration, robot_1_stop_time);
  const Vector2f robot_1_final_stop_position =
      robot_1_position + robot_1_final_stop_delta_position;

  current_robot_logger->AddCircle(robot_1_final_stop_position,
                                  kRobotRadius + safety_margin / 2, 0, 0, 0, 1);

  if (IsCollidingDuringSlowSlowPeriod(
          current_robot_logger, robot_1_position, robot_1_current_velocity,
          robot_1_deceleration, robot_2_position, robot_2_current_velocity,
          robot_2_deceleration, robot_radius,
          std::min(robot_1_stop_time, robot_2_stop_time), safety_margin)) {
    if (kDebug) {
      current_robot_logger->LogPrint("Slow slow period collision.");
    }
    return true;
  }

  Vector2f stopped_position, current_position, current_velocity;
  const Vector2f& current_acceleration = (robot_1_stop_time < robot_2_stop_time)
                                             ? robot_2_deceleration
                                             : robot_1_deceleration;
  float time_delta = (robot_1_stop_time < robot_2_stop_time)
                         ? robot_2_stop_time - robot_1_stop_time
                         : robot_1_stop_time - robot_2_stop_time;

  if (robot_1_stop_time < robot_2_stop_time) {
    stopped_position =
        robot_1_position + CalculateDeltaPosition(robot_1_current_velocity,
                                                  robot_1_deceleration,
                                                  robot_1_stop_time);
    current_position =
        robot_2_position + CalculateDeltaPosition(robot_2_current_velocity,
                                                  robot_2_deceleration,
                                                  robot_1_stop_time);
    current_velocity =
        robot_2_current_velocity +
        CalculateDeltaVelocity(robot_2_deceleration, robot_1_stop_time);
  } else {
    stopped_position =
        robot_2_position + CalculateDeltaPosition(robot_2_current_velocity,
                                                  robot_2_deceleration,
                                                  robot_2_stop_time);
    current_position =
        robot_1_position + CalculateDeltaPosition(robot_1_current_velocity,
                                                  robot_1_deceleration,
                                                  robot_2_stop_time);
    current_velocity =
        robot_1_current_velocity +
        CalculateDeltaVelocity(robot_1_deceleration, robot_2_stop_time);
  }

  return IsCollidingDuringSlowStopPeriod(current_robot_logger, current_position,
                                         current_velocity, current_acceleration,
                                         stopped_position, robot_radius,
                                         time_delta, safety_margin);
}

bool DSS::PillBoxCollide(Logger* current_robot_logger,
                         const Vector2f& robot_1_position,
                         const Vector2f& robot_1_current_velocity,
                         const Vector2f& robot_1_command_velocity,
                         const motion::MotionModel& robot_1_motion_model,
                         const Vector2f& robot_2_position,
                         const Vector2f& robot_2_current_velocity,
                         const Vector2f& robot_2_command_velocity,
                         const motion::MotionModel& robot_2_motion_model,
                         const float robot_radius, const float control_period,
                         const float activation_lag, const float margin) const {
  static const bool kShowProposedLines = false;
  const Vector2f robot_1_end_position = CalculateEndPositionWithCommand(
      robot_1_position, robot_1_current_velocity, robot_1_command_velocity,
      control_period, robot_1_motion_model);
  const Vector2f robot_1_end_velocity = CalculateEndVelocityWithCommand(
      robot_1_current_velocity, robot_1_command_velocity, control_period,
      robot_2_motion_model);

  const Vector2f robot_2_end_position = CalculateEndPositionWithCommand(
      robot_2_position, robot_2_current_velocity, robot_2_command_velocity,
      control_period, robot_2_motion_model);
  const Vector2f robot_2_end_velocity = CalculateEndVelocityWithCommand(
      robot_2_current_velocity, robot_2_command_velocity, control_period,
      robot_2_motion_model);

  current_robot_logger->AddCircle(robot_1_end_position, kRobotRadius, 1, 1, 1,
                                  0.5);

  if (kShowProposedLines) {
    if (IsCollidingDuringControlPeriod(
            current_robot_logger, robot_1_position, robot_1_current_velocity,
            robot_1_command_velocity, robot_1_motion_model, robot_2_position,
            robot_2_current_velocity, robot_2_command_velocity,
            robot_2_motion_model, robot_radius, control_period + activation_lag,
            margin)) {
      // CYAN lines.
      current_robot_logger->AddLine(robot_1_end_position,
                                    robot_1_position + robot_1_command_velocity,
                                    0, 1, 1, 0.5);
    } else if (IsCollidingDuringHaltingPeriod(
                   current_robot_logger, robot_1_end_position,
                   robot_1_end_velocity, robot_1_motion_model,
                   robot_2_end_position, robot_2_end_velocity,
                   robot_2_motion_model, robot_radius, margin)) {
      // Purple lines.
      current_robot_logger->AddLine(robot_1_end_position,
                                    robot_1_position + robot_1_command_velocity,
                                    1, 0, 1, 0.5);
    }
  }

  return (IsCollidingDuringControlPeriod(
             current_robot_logger, robot_1_position, robot_1_current_velocity,
             robot_1_command_velocity, robot_1_motion_model, robot_2_position,
             robot_2_current_velocity, robot_2_command_velocity,
             robot_2_motion_model, robot_radius,
             control_period + activation_lag, margin)) ||
         (IsCollidingDuringHaltingPeriod(
             current_robot_logger, robot_1_end_position, robot_1_end_velocity,
             robot_1_motion_model, robot_2_end_position, robot_2_end_velocity,
             robot_2_motion_model, robot_radius, margin));
}

// Uses squared Eucilidean distance, as per the DSS paper.
float DSS::Cost(const Vector2f& desired, const Vector2f& postulated) {
  return (postulated - desired).squaredNorm();
}

void DSS::GenerateNewVelocities(const Vector2f& current_velocity,
                                vector<Vector2f>* new_velocities,
                                const size_t num_velocities,
                                const float control_period,
                                const MotionModel& motion_model) {
  for (size_t i = 0; i < num_velocities; ++i) {
    const float rand_x =
        random.UniformRandom(-motion_model.a_max * control_period,
                             motion_model.a_max * control_period);
    const float rand_y =
        random.UniformRandom(-motion_model.a_max * control_period,
                             motion_model.a_max * control_period);
    const Vector2f proposed_addition(rand_x, rand_y);
    const Vector2f proposed_velocity =
        CalculateDeltaVelocity(CapAcceleration(proposed_addition, motion_model),
                               control_period) +
        current_velocity;
    new_velocities->push_back(CapVelocity(proposed_velocity, motion_model));
  }
}

// Generates velocities where we drive exactly perpendicular to the current
// velocity, in an attempt to cheaply add max swerve to the robots.
void GeneratePerpendicularVelocities(const Vector2f& current_velocity,
                                     vector<Vector2f>* new_velocities,
                                     const float control_period,
                                     const MotionModel& motion_model) {
  const Vector2f robot_frame_current_velocity_normed =
      GetNormalizedOrZero(current_velocity) * motion_model.a_max *
      control_period;
  const Vector2f robot_frame_current_velocity_perp(
      -robot_frame_current_velocity_normed.y(),
      robot_frame_current_velocity_normed.x());
  new_velocities->push_back(current_velocity +
                            robot_frame_current_velocity_perp);
  new_velocities->push_back(current_velocity -
                            robot_frame_current_velocity_perp);
}

bool DSS::IsTryingToHalt(const Vector2f& robot_frame_desired_robot_velocity,
                         const SSLVisionId& vision_id) const {
  static constexpr bool kDebug = false;
  if (robot_frame_desired_robot_velocity.squaredNorm() <
      kMovingVelocityStopped) {
    if (kDebug) {
      LOG(WARNING) << "Quitting DSS as we (SSL ID: " << vision_id
                   << ") want to be stopped! Commanded: "
                   << robot_frame_desired_robot_velocity.squaredNorm();
    }
    return true;
  }
  return false;
}

bool DSS::AvoidObstacle(state::SharedRobotState* current_robot,
                        const Vector2f& current_world_robot,
                        const Vector2f& other_world_robot,
                        const Rotation2Df& world_to_current_robot_transform,
                        const Rotation2Df& current_robot_to_world_transform,
                        const Vector2f& robot_frame_current_robot_velocity,
                        const float margin, const float control_period,
                        const MotionModel& motion_model, Logger* logger) const {
  static constexpr bool kDebug = false;
  const float squared_robot_difference =
      (current_world_robot - other_world_robot).squaredNorm();
  // Check to see if the robot is inside the given obstacle. If so, drive away
  // from it.
  // Due to the fact that we terminate above if the current robot is commanding
  // a stopped velocity, we know that the current robot is interested in moving;
  // this ensures that in the future we will be able to come up with
  // propositions that do not all get rejected due to being inside of another
  // robot at the start.
  if (IsInCollision(squared_robot_difference, kRobotRadius, margin)) {
    const Vector2f robot_position_delta =
        current_world_robot - other_world_robot;
    const Vector2f robot_frame_norm_exit_vector =
        world_to_current_robot_transform *
        GetNormalizedOrZero(robot_position_delta);
    const Vector2f robot_frame_desired_exit_velocity =
        robot_frame_norm_exit_vector * kObstacleExitVelocity;
    const Vector2f robot_frame_desired_exit_accel =
        robot_frame_desired_exit_velocity - robot_frame_current_robot_velocity;
    const Vector2f robot_frame_capped_exit_accel =
        CapAcceleration(robot_frame_desired_exit_accel, motion_model);
    const Vector2f robot_frame_safe_exit_velocity =
        robot_frame_current_robot_velocity +
        CalculateDeltaVelocity(robot_frame_capped_exit_accel, control_period);
    if (kDebug) {
      LOG(WARNING) << "Drive out of collision.";
      logger->LogPrint("Drive out of collision.");
      logger->LogPrint("Vel: %f, %f", robot_frame_desired_exit_velocity.x(),
                       robot_frame_desired_exit_velocity.y());
      logger->LogPrint("Accel: %f, %f", robot_frame_desired_exit_accel.x(),
                       robot_frame_desired_exit_accel.y());
      logger->LogPrint("Safe Accel: %f, %f", robot_frame_capped_exit_accel.x(),
                       robot_frame_capped_exit_accel.y());
      logger->LogPrint("Safe vel: %f, %f", robot_frame_safe_exit_velocity.x(),
                       robot_frame_safe_exit_velocity.y());
      logger->AddLine(
          current_world_robot,
          current_world_robot +
              current_robot_to_world_transform * robot_frame_safe_exit_velocity,
          1, 1, 0, 1);
    }

    // Set the new velocity for the bot.
    current_robot->velocity_x = robot_frame_safe_exit_velocity.x();
    current_robot->velocity_y = robot_frame_safe_exit_velocity.y();
    return true;
  }
  return false;
}

void DSS::UpdateState(state::SharedRobotState* current_robot,
                      logger::Logger* current_robot_logger,
                      const state::PositionVelocityState::RobotPositionVelocity&
                          current_world_robot,
                      const motion::MotionModel& current_motion_model,
                      const state::PositionVelocityState::RobotPositionVelocity&
                          other_world_robot,
                      const motion::MotionModel& other_motion_model,
                      const Eigen::Vector2f& other_robot_command_velocity,
                      const float control_period, const float margin) {
  static constexpr bool kDebug = false;
  static constexpr bool kShowProposedLines = false;

  const Vector2f robot_frame_desired_robot_velocity =
      Vector2f(current_robot->velocity_x, current_robot->velocity_y);

  // Check to see if commanded stopped velocity. If so, then ignore all other
  // inputs.
  if (IsTryingToHalt(robot_frame_desired_robot_velocity,
                     current_world_robot.ssl_vision_id)) {
    return;
  }

  Rotation2Df current_robot_to_world_transform(
      current_world_robot.position.angle);
  Rotation2Df world_to_current_robot_transform(
      -current_world_robot.position.angle);
  Rotation2Df other_robot_to_world_transform(other_world_robot.position.angle);

  const Vector2f& robot_frame_current_robot_velocity =
      current_world_robot.velocity.translation;
  const Vector2f world_frame_current_robot_velocity =
      current_robot_to_world_transform * robot_frame_current_robot_velocity;

  if (kDebug) {
    current_robot_logger->LogPrint("RF Curr Vel: %f, %f",
                                   robot_frame_current_robot_velocity.x(),
                                   robot_frame_current_robot_velocity.y());
    current_robot_logger->LogPrint("RF Desir Vel: %f, %f",
                                   robot_frame_desired_robot_velocity.x(),
                                   robot_frame_desired_robot_velocity.y());
  }

  // Avoid static obstacles if we're inside of one.
  if (AvoidObstacle(current_robot, current_world_robot.position.translation,
                    other_world_robot.position.translation,
                    world_to_current_robot_transform,
                    current_robot_to_world_transform,
                    robot_frame_current_robot_velocity, margin, control_period,
                    current_motion_model, current_robot_logger)) {
    return;
  }

  // Short circut if there is no collision.
  if (!PillBoxCollide(
          current_robot_logger, current_world_robot.position.translation,
          world_frame_current_robot_velocity,
          current_robot_to_world_transform * robot_frame_desired_robot_velocity,
          current_motion_model, other_world_robot.position.translation,
          other_robot_to_world_transform *
              other_world_robot.velocity.translation,
          other_robot_command_velocity, other_motion_model, kRobotRadius,
          control_period, kHardwareLagTranslation, margin)) {
    return;
  }

  if (kDebug) {
    LOG(WARNING) << "DSS Engaged!";
  }

  current_robot_logger->AddCircle(current_world_robot.position.translation,
                                  kRobotRadius, 1, 0, 0, 1);

  current_robot_logger->LogPrint("Not safe with robot SSL ID: %d",
                                 other_world_robot.ssl_vision_id);

  current_robot_logger->AddLine(current_world_robot.position.translation,
                                current_world_robot.position.translation +
                                    world_frame_current_robot_velocity,
                                1, 0, 0, 0.5);

  // Generate a new velocity for current bot.
  vector<Vector2f> robot_frame_proposed_velocities;
  // Genrate broad phase velocities.
  GenerateNewVelocities(robot_frame_current_robot_velocity,
                        &robot_frame_proposed_velocities, kNumNewVelocities,
                        control_period, current_motion_model);

  GeneratePerpendicularVelocities(robot_frame_current_robot_velocity,
                                  &robot_frame_proposed_velocities,
                                  control_period, current_motion_model);

  // Default to slam on brakes.
  Vector2f robot_frame_current_best =
      CapVelocity(robot_frame_current_robot_velocity +
                      CalculateDeltaVelocity(
                          CapAcceleration(-robot_frame_current_robot_velocity,
                                          current_motion_model),
                          control_period),
                  current_motion_model);
  // Default to cost of slamming on brakes.
  float robot_frame_current_best_cost =
      Cost(robot_frame_desired_robot_velocity, robot_frame_current_best);
  bool using_default = true;

  // Handle current robot.
  for (const Vector2f& robot_frame_proposed_velocity :
       robot_frame_proposed_velocities) {
    const Vector2f world_frame_proposed_velocity =
        current_robot_to_world_transform * robot_frame_proposed_velocity;
    if (!PillBoxCollide(
            current_robot_logger, current_world_robot.position.translation,
            world_frame_current_robot_velocity, world_frame_proposed_velocity,
            current_motion_model, other_world_robot.position.translation,
            other_robot_to_world_transform *
                other_world_robot.velocity.translation,
            other_robot_to_world_transform * other_robot_command_velocity,
            other_motion_model, kRobotRadius, control_period, 0, margin)) {
      if (kShowProposedLines) {
        current_robot_logger->AddLine(current_world_robot.position.translation,
                                      current_world_robot.position.translation +
                                          world_frame_proposed_velocity,
                                      1, 1, 0, 1);
      }

      const float proposed_cost = Cost(robot_frame_desired_robot_velocity,
                                       robot_frame_proposed_velocity);
      if (proposed_cost < robot_frame_current_best_cost) {
        robot_frame_current_best = robot_frame_proposed_velocity;
        robot_frame_current_best_cost = proposed_cost;
        using_default = false;
      }
    } else {
      if (kShowProposedLines) {
        current_robot_logger->AddLine(current_world_robot.position.translation,
                                      current_world_robot.position.translation +
                                          world_frame_proposed_velocity,
                                      0, 0, 0, 0.2);
        current_robot_logger->AddCircle(
            current_world_robot.position.translation +
                world_frame_current_robot_velocity * control_period,
            kRobotRadius + margin / 2, 1, 0.2, 0.2, 0.2);
      }
    }
  }

  if (!using_default && kDebug) {
    LOG(WARNING) << "Not using default!";
  }

  if (kDebug) {
    current_robot_logger->LogPrint("RF Commanded Vel: %f, %f",
                                   robot_frame_current_best.x(),
                                   robot_frame_current_best.y());
  }

  // Set the new velocity for the bot.
  current_robot->velocity_x = robot_frame_current_best.x();
  current_robot->velocity_y = robot_frame_current_best.y();
}

float GetMargin(const MotionModel& motion_model, const Vector2f& ball_position,
                const Vector2f& robot_position, const Vector2f& velocity) {
  if ((ball_position - robot_position).squaredNorm() <
      Sq(3.0f * kRobotRadius)) {
    return kDSSSafetySmallerMargin;
  }
  return kDSSSafetySmallerMargin +
         (velocity.squaredNorm() / Sq(motion_model.v_max)) * kDSSSafetyMargin;
}

void LogRobotPosition(Logger* logger, const Vector2f& position,
                      const float margin) {
  logger->AddCircle(position, kRobotRadius + margin / 2, 0, 1, 0, 0.5);
}

void DSS::MakeSafe(const MotionModel& our_motion_model,
                   const MotionModel& their_motion_model,
                   const float control_period) {
  vector<SharedRobotState>* shared_states =
      soccer_state_->GetMutableSharedState()->GetMutableSharedStates();

  for (OurRobotIndex robot_index = 0; robot_index < shared_states->size();
       ++robot_index) {
    SharedRobotState& current_robot_shared_state =
        (*shared_states)[robot_index];
    const auto& current_world_robot =
        world_state_.GetOurRobotPosition(robot_index);
    const auto& ball_position = world_state_.GetBallPosition();

    const float margin = GetMargin(our_motion_model, ball_position.position,
                                   current_world_robot.position.translation,
                                   current_world_robot.velocity.translation);

    if (current_world_robot.confidence > 0) {
      Logger* logger =
          soccer_state_->GetMutableRobotLoggerByOurRobotIndex(robot_index);
      LogRobotPosition(logger, current_world_robot.position.translation,
                       margin);
      logger->Push();
      for (const SharedRobotState& other_robot_shared_state : *shared_states) {
        if (other_robot_shared_state.enabled) {
          // Do not compare to self.
          if (&current_robot_shared_state != &other_robot_shared_state) {
            //         logger->LogPrint("Making Safe for Teammate");
            // Extract out other world robot and the other vector.
            const auto& team_world_robot = world_state_.GetOurRobotPosition(
                other_robot_shared_state.our_robot_index);
            if (team_world_robot.confidence > 0) {
              const Vector2f team_world_robot_command_velocity(
                  other_robot_shared_state.velocity_x,
                  other_robot_shared_state.velocity_y);
              UpdateState(
                  &current_robot_shared_state, logger, current_world_robot,
                  our_motion_model, team_world_robot, our_motion_model,
                  team_world_robot_command_velocity, control_period, margin);
            }
          }
        }
      }
      for (const auto& opposing_world_robot : world_state_.GetTheirRobots()) {
        if (opposing_world_robot.confidence > 0) {
          UpdateState(&current_robot_shared_state, logger, current_world_robot,
                      their_motion_model, opposing_world_robot,
                      their_motion_model,
                      opposing_world_robot.velocity.translation, control_period,
                      margin);
        }
      }
      logger->Pop();
    } else {
      // Turn off disabled shared state.
      current_robot_shared_state.enabled = false;
      // Sanitize data, just in case.
      current_robot_shared_state.ssl_vision_id = 0;
      current_robot_shared_state.velocity_x = 0;
      current_robot_shared_state.velocity_y = 0;
    }
  }
}
}  // namespace safety
