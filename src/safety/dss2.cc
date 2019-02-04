// Copyright 2018 kvedder@umass.edu
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

#include "safety/dss2.h"

#include <algorithm>
#include <limits>
#include <string>
#include <tuple>
#include <utility>

#include "constants/constants.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/rectangle_obstacle.h"
#include "safety/dss_helpers.h"
#include "util/array_util.h"

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
DSS2::ObstacleFlagTable DSS2::obstacle_flag_lookup_table_;

// List of things that need to be fixed (updated June 3rd, 2018):
//  - Diagnose why pillbox without modification is shorter than the actual
//  stopping distance in sim.
//    -- Remove the artifical extension hack.

DSS2::DSS2(const WorldState& world_state, SoccerState* soccer_state)
    : random(),
      world_state_(world_state),
      soccer_state_(soccer_state),
      timing_file_("dss_timings.txt", std::ios::out) {
  ResetObstacleFlagTable();
}

std::vector<Position> GetDecelerationSlices(
    const Position& start, const Velocity& start_velocity,
    const Acceleration& decel, const float stop_time,
    const motion::MotionModel& motion_model) {
  static constexpr float kTimeIncreaseAmount = 0.1;
  std::vector<Position> slices;
  return slices;

  for (float s = 0; s < stop_time; s += kTimeIncreaseAmount) {
    slices.push_back(CalculateEndPositionWithAccel(start, start_velocity, decel,
                                                   s, motion_model));
  }
  return slices;
}

Velocity RobotToWorldFrame(const Velocity& v, const float robot_angle) {
  return Rotation2Df(robot_angle) * v;
}

Velocity WorldToRobotFrame(const Velocity& v, const float robot_angle) {
  return Rotation2Df(-robot_angle) * v;
}

float GetVariableDSSSafetyMargin(const DSS2::TrajectoryCheckpoints& checkpoints,
                                 const bool use_variable_margin = true) {
  if (use_variable_margin &&
      checkpoints.initial_velocity.squaredNorm() <
          Sq(kDSSSafetyMarginSwapSpeed)) {
    return kDSSSafetySmallerMargin;
  }
  return kDSSSafetyMargin;
}

void DrawTrajectory(const DSS2::TrajectoryCheckpoints& checkpoints,
                    const motion::MotionModel& model, logger::Logger* logger) {
  const float radius =
      kRobotRadius + GetVariableDSSSafetyMargin(checkpoints) / 2;
  logger->AddCircle(checkpoints.end_control_position,
                    radius + GetVariableDSSSafetyMargin(checkpoints) / 2, 0.57,
                    0.43, 0.85, 1);
  logger->AddCircle(checkpoints.halted_position,
                    radius + GetVariableDSSSafetyMargin(checkpoints) / 2, 0, 0,
                    0, 1);

  const auto slices = GetDecelerationSlices(
      checkpoints.end_control_position, checkpoints.end_control_velocity,
      checkpoints.halting_acceleration,
      checkpoints.halted_time - checkpoints.end_control_time, model);

  static constexpr float kColorChangeAmount = 0.1;
  for (size_t i = 0; i < slices.size(); ++i) {
    const float color_amount =
        std::max(0.0f, 1.0f - static_cast<float>(i + 1) * kColorChangeAmount);
    const Position& p = slices[i];
    logger->AddCircle(p, radius + GetVariableDSSSafetyMargin(checkpoints) / 2,
                      color_amount, color_amount, 0, 1);
  }

  const Vector2f norm_line = geometry::GetNormalizedOrZero(
      Vector2f(checkpoints.end_control_position - checkpoints.halted_position));
  const Vector2f perp_line =
      geometry::Perp(norm_line) *
      (radius + GetVariableDSSSafetyMargin(checkpoints) / 2);
  logger->AddLine(checkpoints.end_control_position + perp_line,
                  checkpoints.halted_position + perp_line, 0.57, 0.43, 0.85, 1);
  logger->AddLine(checkpoints.end_control_position - perp_line,
                  checkpoints.halted_position - perp_line, 0.57, 0.43, 0.85, 1);
}

void DrawTrajectories(const DSS2::TrajectoryArray& array,
                      const motion::MotionModel& model,
                      logger::Logger* logger) {
  for (const auto& t : array) {
    DrawTrajectory(t, model, logger);
  }
}

void SanityCheckCollisions(const DSS2::TrajectoryArray& our_trajectories,
                           logger::Logger* logger) {
  for (const DSS2::TrajectoryCheckpoints& t1 : our_trajectories) {
    for (const DSS2::TrajectoryCheckpoints& t2 : our_trajectories) {
      if (&t1 == &t2) {
        continue;
      }
      if (IsInCollision((t1.halted_position - t2.halted_position).squaredNorm(),
                        t1.radius, t2.radius, GetVariableDSSSafetyMargin(t1))) {
        logger->LogPrint("ERROR: Stopped IDs collide: %d & %d",
                         t1.ssl_vision_id, t2.ssl_vision_id);
      }
    }
  }
}

void DSS2::UpdateSharedState(const DSS2::TrajectoryArray& our_trajectories) {
  std::vector<SharedRobotState>& shared_states =
      *soccer_state_->GetMutableSharedState()->GetMutableSharedStates();

  size_t i = 0;
  for (SharedRobotState& command_info : shared_states) {
    if (!command_info.enabled ||
        world_state_.GetOurRobotPosition(command_info.our_robot_index)
                .confidence <= 0) {
      continue;
    }

    const auto& replacement_trajectory = our_trajectories.Get(i);

    NP_CHECK(command_info.ssl_vision_id ==
             replacement_trajectory.ssl_vision_id);

    const auto current_info =
        world_state_.GetOurRobotPosition(command_info.our_robot_index);

    NP_CHECK(
        !std::isnan(replacement_trajectory.initial_commanded_acceleration.x()));
    NP_CHECK(
        !std::isnan(replacement_trajectory.initial_commanded_acceleration.y()));

    command_info.velocity_x = 0;
    command_info.velocity_y = 0;

    if (command_info.acceleration_command.translation !=
        replacement_trajectory.initial_commanded_acceleration) {
      command_info.acceleration_command.translation =
          replacement_trajectory.initial_commanded_acceleration;
      command_info.dss_changed_command = true;
    }
    ++i;
  }
}

bool TryingToBeStopped(const Acceleration& commanded_accel) {
  return (commanded_accel.squaredNorm() < Sq(50));
}

bool SkippableTactic(const tactics::TacticIndex& tactic) {
  switch (tactic) {
    case tactics::TacticIndex::PRIMARY_ATTACKER:
    case tactics::TacticIndex::GOALIE:
      return true;
    default:
      return false;
  }
}

const bool ShouldDSSOurTeam(const obstacle::ObstacleFlag& obstacle_flag) {
  return (obstacle_flag & obstacle::ObstacleFlag::GetOurRobots())
      .GetFlags()
      .any();
}

const bool ShouldDSSTheirTeam(const obstacle::ObstacleFlag& obstacle_flag) {
  return (obstacle_flag & obstacle::ObstacleFlag::GetOpponentRobots())
      .GetFlags()
      .any();
}

const bool ShouldDSSBall(const obstacle::ObstacleFlag& obstacle_flag) {
  return (obstacle_flag & obstacle::ObstacleFlag::GetAllBalls())
      .GetFlags()
      .any();
}

const bool ShouldDSSTheirDefenseArea(
    const obstacle::ObstacleFlag& obstacle_flag) {
  return (obstacle_flag & obstacle::ObstacleFlag::GetTheirDefenseArea())
      .GetFlags()
      .any();
}

const bool ShouldDSSOurDefenseArea(
    const obstacle::ObstacleFlag& obstacle_flag) {
  return (obstacle_flag & obstacle::ObstacleFlag::GetOurDefenseArea())
      .GetFlags()
      .any();
}

std::string BoolToString(const bool b) { return (b) ? "true" : "false"; }

void LogSkippedSections(const OurRobotIndex& i,
                        const tactics::TacticIndex& current_tactic,
                        const obstacle::ObstacleFlag& obstacle_flag,
                        logger::Logger* logger) {
  const auto dss_our_team = BoolToString(!SkippableTactic(current_tactic) &&
                                         ShouldDSSOurTeam(obstacle_flag));
  const auto dss_their_team = BoolToString(!SkippableTactic(current_tactic) &&
                                           ShouldDSSTheirTeam(obstacle_flag));
  const auto dss_ball = BoolToString(ShouldDSSBall(obstacle_flag));
  const auto dss_their_defense_area =
      BoolToString(ShouldDSSTheirDefenseArea(obstacle_flag));
  const auto dss_our_defense_area =
      BoolToString(ShouldDSSOurDefenseArea(obstacle_flag));
  logger->LogPrint(
      "Robot index %d status: DSS our team: %s, their team: %s, ball: %s their "
      "defense area: %s, our defense area: %s",
      i, dss_our_team, dss_their_team, dss_ball, dss_their_defense_area,
      dss_our_defense_area);
}

void DSS2::MakeSafe(const motion::MotionModel& our_motion_model,
                    const motion::MotionModel& their_motion_model,
                    logger::Logger* logger) {
  static constexpr bool kDebug = false;
  logger->Push("DSS");
  const auto before_make_safe = GetMonotonicTime();
  std::pair<DSS2::TrajectoryArray, DSS2::TrajectoryArray>
      our_their_trajectories =
          GetOurTheirTrajectories(our_motion_model, their_motion_model, logger);
  DSS2::TrajectoryArray& our_trajectories = our_their_trajectories.first;
  const DSS2::TrajectoryArray& their_trajectories =
      our_their_trajectories.second;
  const auto ball_motion = ComputeBallMotion();

  // Handle other robots.
  for (OurRobotIndex i = 0; i < our_trajectories.GetElementCount(); ++i) {
    DSS2::TrajectoryCheckpoints& trajectory_in_question =
        *our_trajectories.GetMutable(i);
    const tactics::TacticIndex& current_tactic =
        soccer_state_->GetRobotByOurRobotIndex(i).current_tactic_;

    NP_CHECK(i < obstacle_flag_lookup_table_.size());
    LogSkippedSections(i, current_tactic, obstacle_flag_lookup_table_[i],
                       logger);

    if (TryingToBeStopped(
            trajectory_in_question.initial_commanded_acceleration)) {
      continue;
    }

    if (!TrajectoryCollidesWithWorld(
            trajectory_in_question, current_tactic, our_trajectories,
            our_motion_model, their_trajectories, their_motion_model,
            trajectory_in_question.dss_obstacles, logger)) {
      continue;
    }

    if (RepairStartInCollision(
            &trajectory_in_question, current_tactic, our_trajectories,
            our_motion_model, their_trajectories, their_motion_model,
            ball_motion.first, trajectory_in_question.dss_obstacles, logger)) {
      logger->LogPrint("SSL ID: %d moving out of collision in start",
                       trajectory_in_question.ssl_vision_id);
      continue;
    }

    if (AttemptAlternateCommands(
            &trajectory_in_question, current_tactic, our_trajectories,
            our_motion_model, their_trajectories, their_motion_model,
            trajectory_in_question.dss_obstacles, logger)) {
      logger->LogPrint("Alternate found; SSL ID: %d",
                       trajectory_in_question.ssl_vision_id);
    } else {
      trajectory_in_question =
          MakeHaltingTrajectory(trajectory_in_question, our_motion_model);
      logger->LogPrint("No alternte found; halting SSL ID: %d",
                       trajectory_in_question.ssl_vision_id);
    }
  }

  SanityCheckCollisions(our_trajectories, logger);

  if (kDebug) {
    DrawTrajectories(our_trajectories, our_motion_model, logger);
    DrawTrajectories(their_trajectories, their_motion_model, logger);
    DrawTrajectory(ball_motion.first, ball_motion.second, logger);
  }

  UpdateSharedState(our_trajectories);

  logger->Pop();
  ResetObstacleFlagTable();
  const auto after_make_safe = GetMonotonicTime();
  if (kDSSDumpTimings && timing_file_.is_open()) {
    timing_file_ << (after_make_safe - before_make_safe) * 1000.0 << '\n';
  }
}

DSS2::TrajectoryCheckpoints ZeroAnyMovement(
    const DSS2::TrajectoryCheckpoints& checkpoints) {
  static const Eigen::Vector2f kZero(0, 0);
  DSS2::TrajectoryCheckpoints modified_checkpoints = checkpoints;
  modified_checkpoints.initial_velocity = kZero;
  modified_checkpoints.initial_commanded_acceleration = kZero;

  modified_checkpoints.end_control_position =
      modified_checkpoints.initial_position;
  modified_checkpoints.end_control_velocity = kZero;
  modified_checkpoints.end_control_time = 0;

  modified_checkpoints.halted_position = modified_checkpoints.initial_position;
  modified_checkpoints.halted_time = 0;
  modified_checkpoints.halting_acceleration = kZero;
  return modified_checkpoints;
}

bool DSS2::RepairStartInCollision(
    DSS2::TrajectoryCheckpoints* trajectory_in_question,
    const tactics::TacticIndex& trajectory_in_question_tactic,
    const DSS2::TrajectoryArray& our_trajectories,
    const MotionModel& our_motion_model,
    const DSS2::TrajectoryArray& their_trajectories,
    const MotionModel& their_motion_model,
    const DSS2::TrajectoryCheckpoints& ball_trajectory,
    const obstacle::ObstacleFlag& obstacle_flag, logger::Logger* logger) const {
  const float kLeaveAccelMagnitude = our_motion_model.a_max / 2.0f;
  const bool skippable_tactic = SkippableTactic(trajectory_in_question_tactic);
  const bool dss_our_team = ShouldDSSOurTeam(obstacle_flag);
  const bool dss_their_team = ShouldDSSTheirTeam(obstacle_flag);
  const bool dss_ball = ShouldDSSBall(obstacle_flag);
  const bool dss_their_defense_area = ShouldDSSTheirDefenseArea(obstacle_flag);
  const bool dss_our_defense_area = ShouldDSSOurDefenseArea(obstacle_flag);

  // Our trajectories.
  if (!skippable_tactic && dss_our_team) {
    for (const DSS2::TrajectoryCheckpoints& other_trajectory :
         our_trajectories) {
      if (trajectory_in_question == &other_trajectory) {
        continue;
      }
      if (IsInCollision((trajectory_in_question->initial_position -
                         other_trajectory.initial_position)
                            .squaredNorm(),
                        trajectory_in_question->radius, other_trajectory.radius,
                        kDSSSafetyMargin)) {
        const Acceleration escape_accel =
            GetNormalizedOrZero(
                Vector2f(trajectory_in_question->initial_position -
                         other_trajectory.initial_position)) *
            kLeaveAccelMagnitude;
        *trajectory_in_question = MakeNewAccelerationTrajectory(
            *trajectory_in_question, our_motion_model, escape_accel);
        logger->Push("Start in collision:");
        logger->LogPrint("Collides with our team");
        logger->Pop();
        return true;
      }
    }
  }

  // Their trajectories.
  if (!skippable_tactic && dss_their_team) {
    for (const DSS2::TrajectoryCheckpoints& other_trajectory :
         their_trajectories) {
      if (IsInCollision((trajectory_in_question->initial_position -
                         other_trajectory.initial_position)
                            .squaredNorm(),
                        trajectory_in_question->radius, other_trajectory.radius,
                        GetVariableDSSSafetyMargin(*trajectory_in_question))) {
        const Acceleration escape_accel =
            GetNormalizedOrZero(
                Vector2f(trajectory_in_question->initial_position -
                         other_trajectory.initial_position)) *
            kLeaveAccelMagnitude;
        *trajectory_in_question = MakeNewAccelerationTrajectory(
            *trajectory_in_question, our_motion_model, escape_accel);
        logger->Push("Start in collision:");
        logger->LogPrint("Collides with their team");
        logger->Pop();
        return true;
      }
    }
  }

  // Ball trajectory.
  if (dss_ball) {
    if (IsInCollision((trajectory_in_question->initial_position -
                       ball_trajectory.initial_position)
                          .squaredNorm(),
                      trajectory_in_question->radius, ball_trajectory.radius,
                      kDSSSafetyMargin)) {
      const Acceleration escape_accel =
          GetNormalizedOrZero(
              Vector2f(trajectory_in_question->initial_position -
                       ball_trajectory.initial_position)) *
          kLeaveAccelMagnitude;
      *trajectory_in_question = MakeNewAccelerationTrajectory(
          *trajectory_in_question, our_motion_model, escape_accel);
      logger->Push("Start in collision:");
      logger->LogPrint("Collides with ball");
      logger->Pop();
      return true;
    }
  }

  // Our Defense Area.
  if (dss_our_defense_area) {
    for (const auto* obstacle : obstacle::ObstacleFlag::GetOurDefenseArea()) {
      if (obstacle->PointCollision(trajectory_in_question->initial_position,
                                   kDSSSafetyMargin)) {
        logger->AddCircle(trajectory_in_question->initial_position, 100, 1, 0,
                          0, 1.0);
        const Position nearest_free = obstacle->NearestFreePoint(
            trajectory_in_question->initial_position, kDSSSafetyMargin);
        const Acceleration escape_accel =
            GetNormalizedOrZero(Vector2f(
                nearest_free - trajectory_in_question->initial_position)) *
            kLeaveAccelMagnitude;
        *trajectory_in_question = MakeNewAccelerationTrajectory(
            *trajectory_in_question, our_motion_model, escape_accel);
        logger->Push("Start in collision:");
        logger->LogPrint("Collides with our defense area");
        logger->Pop();
        return true;
      }
    }
  }

  // Their Defense Area.
  if (dss_their_defense_area) {
    for (const auto* obstacle : obstacle::ObstacleFlag::GetTheirDefenseArea()) {
      if (obstacle->PointCollision(trajectory_in_question->initial_position,
                                   kDSSSafetyMargin)) {
        logger->AddCircle(trajectory_in_question->initial_position, 200, 1, 0,
                          1, 1.0);
        const Position nearest_free = obstacle->NearestFreePoint(
            trajectory_in_question->initial_position, kDSSSafetyMargin);
        const Acceleration escape_accel =
            GetNormalizedOrZero(Vector2f(
                nearest_free - trajectory_in_question->initial_position)) *
            kLeaveAccelMagnitude;
        *trajectory_in_question = MakeNewAccelerationTrajectory(
            *trajectory_in_question, our_motion_model, escape_accel);
        logger->Push("Start in collision:");
        logger->LogPrint("Collides with their defense area");
        logger->Pop();
        return true;
      }
    }
  }

  return false;
}

bool DSS2::TrajectoryCollidesWithWorld(
    const DSS2::TrajectoryCheckpoints& trajectory_in_question,
    const tactics::TacticIndex& trajectory_in_question_tactic,
    const DSS2::TrajectoryArray& our_trajectories,
    const MotionModel& our_motion_model,
    const DSS2::TrajectoryArray& their_trajectories,
    const MotionModel& their_motion_model,
    const obstacle::ObstacleFlag& obstacle_flag, logger::Logger* logger) const {
  const bool skippable_tactic = SkippableTactic(trajectory_in_question_tactic);
  const bool dss_our_team = ShouldDSSOurTeam(obstacle_flag);
  const bool dss_their_team = ShouldDSSTheirTeam(obstacle_flag);
  const bool dss_ball = ShouldDSSBall(obstacle_flag);
  const bool dss_their_defense_area = ShouldDSSTheirDefenseArea(obstacle_flag);
  const bool dss_our_defense_area = ShouldDSSOurDefenseArea(obstacle_flag);

  if (!skippable_tactic && dss_our_team &&
      TrajectoryCollidesWithTrajectoryArray(trajectory_in_question,
                                            our_motion_model, our_trajectories,
                                            our_motion_model, logger, false)) {
    return true;
  }

  if (!skippable_tactic && dss_their_team &&
      TrajectoryCollidesWithTrajectoryArray(
          trajectory_in_question, our_motion_model, their_trajectories,
          their_motion_model, logger, true)) {
    return true;
  }

  if (dss_ball) {
    const auto ball_motion = ComputeBallMotion();
    const auto& ball_trajectory = ball_motion.first;
    const auto& ball_motion_model = ball_motion.second;
    if (TrajectoryPairCollide(ball_trajectory, ball_motion_model,
                              trajectory_in_question, our_motion_model,
                              kDSSSafetyMargin, logger)) {
      return true;
    }
  }

  if (dss_their_defense_area) {
    for (const auto* obstacle :
         (obstacle::ObstacleFlag::GetTheirDefenseArea() & obstacle_flag)) {
      if (TrajectorCollideWithObstacle(obstacle, trajectory_in_question,
                                       our_motion_model, kDSSSafetyMargin,
                                       logger)) {
        return true;
      }
    }
  }

  if (dss_our_defense_area) {
    for (const auto* obstacle :
         (obstacle::ObstacleFlag::GetOurDefenseArea() & obstacle_flag)) {
      if (TrajectorCollideWithObstacle(obstacle, trajectory_in_question,
                                       our_motion_model, kDSSSafetyMargin,
                                       logger)) {
        return true;
      }
    }
  }

  return false;
}

std::pair<DSS2::TrajectoryCheckpoints, MotionModel> DSS2::ComputeBallMotion()
    const {
  const InitialCommandInfo ball_info(
      std::numeric_limits<unsigned int>::max(), kDSSBallRadius,
      obstacle::ObstacleFlag::GetFull(),
      world_state_.GetBallPosition().position,
      world_state_.GetBallPosition().velocity, {0, 0});
  const motion::MotionModel ball_motion_model(
      0.01f, std::numeric_limits<float>::max() - 1.0f);
  return {ComputeTrajectoryCheckpoints(ball_info, ball_motion_model),
          ball_motion_model};
}

DSS2::TrajectoryCheckpoints DSS2::MakeNewAccelerationTrajectory(
    const DSS2::TrajectoryCheckpoints& existing_trajectory,
    const MotionModel& motion_model, const Acceleration& new_accel) const {
  InitialCommandInfo halt_command_info(
      existing_trajectory.ssl_vision_id, existing_trajectory.radius,
      existing_trajectory.dss_obstacles, existing_trajectory.initial_position,
      existing_trajectory.initial_velocity, new_accel);
  halt_command_info.Verify();
  return ComputeTrajectoryCheckpoints(halt_command_info, motion_model);
}

DSS2::TrajectoryCheckpoints DSS2::MakeHaltingTrajectory(
    const DSS2::TrajectoryCheckpoints& existing_trajectory,
    const motion::MotionModel& motion_model) const {
  const Acceleration immediate_halt_acceleration = CapAcceleration(
      GetNormalizedOrZero(existing_trajectory.initial_velocity) *
          -motion_model.a_max,
      motion_model);
  return MakeNewAccelerationTrajectory(existing_trajectory, motion_model,
                                       immediate_halt_acceleration);
}

bool DSS2::TrajectoryCollidesWithTrajectoryArray(
    const DSS2::TrajectoryCheckpoints& trajectory,
    const motion::MotionModel& trajectory_motion_model,
    const DSS2::TrajectoryArray& array,
    const motion::MotionModel& array_motion_model, logger::Logger* logger,
    const bool use_variable_margin) const {
  for (const DSS2::TrajectoryCheckpoints& reference_trajectory : array) {
    // Skip trajectories intended for the same robot as the the given one.
    if (trajectory.ssl_vision_id == reference_trajectory.ssl_vision_id) {
      continue;
    }
    if (TrajectoryPairCollide(trajectory, trajectory_motion_model,
                              reference_trajectory, array_motion_model,
                              GetVariableDSSSafetyMargin(reference_trajectory,
                                                         use_variable_margin),
                              logger)) {
      return true;
    }
  }
  return false;
}

void VerifyEndPosition(const Position& end_position,
                       const Position& start_control_position,
                       const Velocity& start_control_velocity,
                       const Acceleration& start_control_acceleration,
                       const float stop_time, const motion::MotionModel& model,
                       logger::Logger* logger) {
  if (!kProduction) {
    const auto computed_end_position = CalculateEndPositionWithAccel(
        start_control_position, start_control_velocity,
        start_control_acceleration, stop_time, model);
    if ((end_position - computed_end_position).squaredNorm() > kEpsilon) {
      LOG(ERROR) << "Given: " << end_position.transpose()
                 << " Computed: " << computed_end_position.transpose();
      logger->AddCircle(end_position, 20, 1, 1, 1, 1);
      logger->AddCircle(end_position, 15, 1, 1, 1, 1);
      logger->AddCircle(computed_end_position, 20, 1, 1, 0, 1);
      logger->AddCircle(computed_end_position, 15, 1, 1, 0, 1);
    }
  }
}

bool ControlPeriodsCollide(
    const Position& p1_start, const Velocity& p1_velocity,
    const Acceleration& p1_acceleration, const Position& p1_end,
    const float p1_radius, const Position& p2_start,
    const Velocity& p2_velocity, const Acceleration& p2_acceleration,
    const Position& p2_end, const float p2_radius, const float control_time,
    const motion::MotionModel& p1_model, const motion::MotionModel& p2_model,
    const float robot_safety_margin, logger::Logger* logger) {
  const Position rel_start = p1_start - p2_start;
  const Velocity rel_velocity = p1_velocity - p2_velocity;
  const Acceleration rel_acceleration = p1_acceleration - p2_acceleration;

  VerifyEndPosition(p1_end, p1_start, p1_velocity, p1_acceleration,
                    control_time, p1_model, logger);
  VerifyEndPosition(p2_end, p2_start, p2_velocity, p2_acceleration,
                    control_time, p2_model, logger);
  NP_CHECK(control_time >= 0);

  const float rel_closest_time = ClampedClosestDistanceTime(
      rel_start, rel_velocity, rel_acceleration, 0.0f, control_time);
  const float rel_closest_distance = DistanceSquared(
      rel_start, rel_velocity, rel_acceleration, rel_closest_time);
  const float rel_start_distance =
      DistanceSquared(rel_start, rel_velocity, rel_acceleration, 0.0f);
  const float rel_end_distance =
      DistanceSquared(rel_start, rel_velocity, rel_acceleration, control_time);
  const float rel_precomputed_end_distance = (p1_end - p2_end).squaredNorm();
  const float rel_middle_distance = DistanceSquared(
      rel_start, rel_velocity, rel_acceleration, control_time / 2.0f);

  return IsInCollision(rel_closest_distance, p1_radius, p2_radius,
                       robot_safety_margin) ||
         IsInCollision(rel_precomputed_end_distance, p1_radius, p2_radius,
                       robot_safety_margin) ||
         IsInCollision(rel_start_distance, p1_radius, p2_radius,
                       robot_safety_margin) ||
         IsInCollision(rel_end_distance, p1_radius, p2_radius,
                       robot_safety_margin) ||
         IsInCollision(rel_middle_distance, p1_radius, p2_radius,
                       robot_safety_margin);
}

bool SlowSlowPeriodsCollide(
    const Position& p1_start, const Velocity& p1_velocity,
    const Acceleration& p1_acceleration, const float p1_stop_time,
    const float p1_radius, const Position& p2_start,
    const Velocity& p2_velocity, const Acceleration& p2_acceleration,
    const float p2_stop_time, const float p2_radius,
    const float robot_safety_margin, logger::Logger* logger) {
  NP_CHECK(p1_stop_time >= 0.0f);
  NP_CHECK(p2_stop_time >= 0.0f);
  // Ensure that the given acceleration is in the opposite direction of the
  // acceleration.
  NP_CHECK_MSG(p1_velocity.dot(p1_acceleration) <= 0,
               "p1 vel: " << p1_velocity.x() << ", " << p1_velocity.y()
                          << " p1 accel: " << p1_acceleration.x() << ", "
                          << p1_acceleration.y());
  NP_CHECK_MSG(p2_velocity.dot(p2_acceleration) <= 0,
               "p2 vel: " << p2_velocity.x() << ", " << p2_velocity.y()
                          << " p2 accel: " << p2_acceleration.x() << ", "
                          << p2_acceleration.y());
  const float min_time = std::min(p1_stop_time, p2_stop_time);

  const Position rel_start = p1_start - p2_start;
  const Velocity rel_velocity = p1_velocity - p2_velocity;
  const Acceleration rel_acceleration = p1_acceleration - p2_acceleration;

  const float rel_closest_time = ClampedClosestDistanceTime(
      rel_start, rel_velocity, rel_acceleration, 0, min_time);
  const float rel_closest_distance = DistanceSquared(
      rel_start, rel_velocity, rel_acceleration, rel_closest_time);
  const float rel_start_distance =
      DistanceSquared(rel_start, rel_velocity, rel_acceleration, 0);
  const float rel_end_distance =
      DistanceSquared(rel_start, rel_velocity, rel_acceleration, min_time);

  return IsInCollision(rel_closest_distance, p1_radius, p2_radius,
                       robot_safety_margin) ||
         IsInCollision(rel_start_distance, p1_radius, p2_radius,
                       robot_safety_margin) ||
         IsInCollision(rel_end_distance, p1_radius, p2_radius,
                       robot_safety_margin);
}

void VerifyEndpointsLineUp(const Position& moving_end_control_position,
                           const Velocity& moving_end_control_velocity,
                           const Acceleration& moving_halting_acceleration,
                           const Position& moving_halt_position,
                           const Time time,
                           const motion::MotionModel& motion_model) {
  if (!kProduction) {
    const Position calculated_end_position = CalculateEndPositionWithAccel(
        moving_end_control_position, moving_end_control_velocity,
        moving_halting_acceleration, time, motion_model);
    NP_CHECK_MSG(
        (moving_halt_position - calculated_end_position).squaredNorm() < 0.1f,
        "Diff: "
            << (moving_halt_position - calculated_end_position).squaredNorm());
  }
}

// Takes the stopped position of the stopped robot, as well as the position,
// velocity, and halting acceleration of the robot that is still running. It
// clamps the closest position between the two stop times, in order to find
// possible collisions.
bool SlowStopPeriodsCollide(const Position& stopped_position,
                            const float stopped_radius,
                            const Position& moving_end_control_position,
                            const Velocity& moving_end_control_velocity,
                            const Acceleration& moving_halting_acceleration,
                            const Position& moving_halt_position,
                            const float moving_radius, const Time longer_time,
                            const Time shorter_time,
                            const float robot_safety_margin,
                            const motion::MotionModel& motion_model,
                            logger::Logger* logger) {
  NP_CHECK(longer_time >= shorter_time);
  VerifyEndpointsLineUp(moving_end_control_position,
                        moving_end_control_velocity,
                        moving_halting_acceleration, moving_halt_position,
                        longer_time, motion_model);
  const Position rel_start = moving_end_control_position - stopped_position;
  const Velocity& rel_velocity = moving_end_control_velocity;
  const Acceleration& rel_acceleration = moving_halting_acceleration;

  const float rel_closest_time = ClampedClosestDistanceTime(
      rel_start, rel_velocity, rel_acceleration, shorter_time, longer_time);
  NP_CHECK(rel_closest_time <= longer_time);
  NP_CHECK(rel_closest_time >= shorter_time);
  const float rel_closest_distance = DistanceSquared(
      rel_start, rel_velocity, rel_acceleration, rel_closest_time);
  const float rel_start_distance =
      DistanceSquared(rel_start, rel_velocity, rel_acceleration, shorter_time);

  const bool in_collision =
      IsInCollision(rel_closest_distance, stopped_radius, moving_radius,
                    robot_safety_margin) ||
      IsInCollision(rel_start_distance, stopped_radius, moving_radius,
                    robot_safety_margin) ||
      IsInCollision((stopped_position - moving_halt_position).squaredNorm(),
                    stopped_radius, moving_radius, robot_safety_margin);

  return in_collision;
}

bool DSS2::TrajectorCollideWithObstacle(const obstacle::Obstacle* obstacle,
                                        const DSS2::TrajectoryCheckpoints& t1,
                                        const MotionModel& t1_motion_model,
                                        const float robot_safety_margin,
                                        Logger* logger) const {
  return obstacle->LineCollision(t1.end_control_position, t1.halted_position,
                                 robot_safety_margin) ||
         obstacle->LineCollision(t1.initial_position, t1.end_control_position,
                                 robot_safety_margin);
}

bool DSS2::TrajectoryPairCollide(const DSS2::TrajectoryCheckpoints& t1,
                                 const motion::MotionModel& t1_motion_model,
                                 const DSS2::TrajectoryCheckpoints& t2,
                                 const motion::MotionModel& t2_motion_model,
                                 const float robot_safety_margin,
                                 logger::Logger* logger) const {
  const Position& p1_initial_position = t1.initial_position;
  const Velocity& p1_initial_velocity = t1.initial_velocity;
  const Acceleration p1_command_acceleration =
      t1.initial_commanded_acceleration;
  const Position& p1_end_control_position = t1.end_control_position;
  const float& p1_radius = t1.radius;

  const Position& p2_initial_position = t2.initial_position;
  const Velocity& p2_initial_velocity = t2.initial_velocity;
  const Acceleration p2_command_acceleration =
      t2.initial_commanded_acceleration;
  const Position& p2_end_control_position = t2.end_control_position;
  const float& p2_radius = t2.radius;

  NP_CHECK(fabs(t1.end_control_time - t2.end_control_time) < kEpsilon);
  const float& control_time = t1.end_control_time;
  if (ControlPeriodsCollide(p1_initial_position, p1_initial_velocity,
                            p1_command_acceleration, p1_end_control_position,
                            p1_radius, p2_initial_position, p2_initial_velocity,
                            p2_command_acceleration, p2_end_control_position,
                            p2_radius, control_time, t1_motion_model,
                            t2_motion_model, robot_safety_margin, logger)) {
    NP_CHECK_MSG(
        ControlPeriodsCollide(
            p2_initial_position, p2_initial_velocity, p2_command_acceleration,
            p2_end_control_position, p2_radius, p1_initial_position,
            p1_initial_velocity, p1_command_acceleration,
            p1_end_control_position, p1_radius, control_time, t2_motion_model,
            t1_motion_model, robot_safety_margin, logger),
        "ID " << t1.ssl_vision_id << " vs " << t2.ssl_vision_id);
    return true;
  } else {
    NP_CHECK_MSG(
        !ControlPeriodsCollide(
            p2_initial_position, p2_initial_velocity, p2_command_acceleration,
            p2_end_control_position, p2_radius, p1_initial_position,
            p1_initial_velocity, p1_command_acceleration,
            p1_end_control_position, p1_radius, control_time, t2_motion_model,
            t1_motion_model, robot_safety_margin, logger),
        "ID " << t1.ssl_vision_id << " vs " << t2.ssl_vision_id);
  }

  const Velocity& p1_end_control_velocity = t1.end_control_velocity;
  const Acceleration& p1_slow_acceleration = t1.halting_acceleration;
  const float p1_stop_time = t1.halted_time - t1.end_control_time;

  const Velocity& p2_end_control_velocity = t2.end_control_velocity;
  const Acceleration& p2_slow_acceleration = t2.halting_acceleration;
  const float p2_stop_time = t2.halted_time - t2.end_control_time;

  if (SlowSlowPeriodsCollide(p1_end_control_position, p1_end_control_velocity,
                             p1_slow_acceleration, p1_stop_time, p1_radius,
                             p2_end_control_position, p2_end_control_velocity,
                             p2_slow_acceleration, p2_stop_time, p2_radius,
                             robot_safety_margin, logger)) {
    NP_CHECK(SlowSlowPeriodsCollide(
        p2_end_control_position, p2_end_control_velocity, p2_slow_acceleration,
        p2_stop_time, p2_radius, p1_end_control_position,
        p1_end_control_velocity, p1_slow_acceleration, p1_stop_time, p1_radius,
        robot_safety_margin, logger));
    return true;
  } else {
    NP_CHECK(!SlowSlowPeriodsCollide(
        p2_end_control_position, p2_end_control_velocity, p2_slow_acceleration,
        p2_stop_time, p2_radius, p1_end_control_position,
        p1_end_control_velocity, p1_slow_acceleration, p1_stop_time, p1_radius,
        robot_safety_margin, logger));
  }

  Position slower_robot_stopped_position(0, 0);
  float slower_radius(0);
  float faster_radius(0);
  Time longer_time_to_stop(0);
  Time shorter_time_to_stop(0);
  Position faster_robot_moving_position(0, 0);
  Velocity faster_robot_moving_velocity(0, 0);
  Acceleration faster_robot_moving_acceleration(0, 0);
  Position faster_robot_halt_position(0, 0);
  motion::MotionModel faster_motion_model(0, 0);
  if (p1_stop_time > p2_stop_time) {
    // t2 stopped before t1.
    // t1 is moving.
    // t2 is stationary.
    VerifyEndPosition(t2.halted_position, p2_end_control_position,
                      p2_end_control_velocity, p2_slow_acceleration,
                      p2_stop_time, t2_motion_model, logger);
    shorter_time_to_stop = p2_stop_time;
    slower_robot_stopped_position = t2.halted_position;
    slower_radius = t2.radius;

    faster_motion_model = t1_motion_model;

    longer_time_to_stop = p1_stop_time;
    faster_robot_moving_position = p1_end_control_position;
    faster_robot_moving_velocity = p1_end_control_velocity;
    faster_robot_moving_acceleration = p1_slow_acceleration;
    faster_robot_halt_position = t1.halted_position;
    faster_radius = t1.radius;
  } else /*(p1_stop_time <= p2_stop_time)*/ {
    // t1 stopped before t2.
    // t2 is moving.
    // t1 is stationary.
    VerifyEndPosition(t1.halted_position, p1_end_control_position,
                      p1_end_control_velocity, p1_slow_acceleration,
                      p1_stop_time, t1_motion_model, logger);
    shorter_time_to_stop = p1_stop_time;
    slower_robot_stopped_position = t1.halted_position;
    slower_radius = t1.radius;

    faster_motion_model = t2_motion_model;

    longer_time_to_stop = p2_stop_time;
    faster_robot_moving_position = p2_end_control_position;
    faster_robot_moving_velocity = p2_end_control_velocity;
    faster_robot_moving_acceleration = p2_slow_acceleration;
    faster_robot_halt_position = t2.halted_position;
    faster_radius = t2.radius;
  }

  if (SlowStopPeriodsCollide(
          slower_robot_stopped_position, slower_radius,
          faster_robot_moving_position, faster_robot_moving_velocity,
          faster_robot_moving_acceleration, faster_robot_halt_position,
          faster_radius, longer_time_to_stop, shorter_time_to_stop,
          robot_safety_margin, faster_motion_model, logger)) {
    return true;
  }

  return false;
}

std::pair<DSS2::TrajectoryArray, DSS2::TrajectoryArray>
DSS2::GetOurTheirTrajectories(const motion::MotionModel& our_motion_model,
                              const motion::MotionModel& their_motion_model,
                              logger::Logger* logger) const {
  InitialCommandInfoArray our_robots;
  const auto shared_state_vector =
      *soccer_state_->GetMutableSharedState()->GetMutableSharedStates();
  for (OurRobotIndex i = 0; i < shared_state_vector.size(); ++i) {
    NP_CHECK(i < kMaxTeamRobots);
    const SharedRobotState& command_info = shared_state_vector[i];
    if (!command_info.enabled ||
        world_state_.GetOurRobotPosition(command_info.our_robot_index)
                .confidence <= 0) {
      continue;
    }

    const auto current_info =
        world_state_.GetOurRobotPosition(command_info.our_robot_index);
    const Velocity& current_velocity = RobotToWorldFrame(
        current_info.velocity.translation, current_info.position.angle);
    const InitialCommandInfo our_robot_command_info(
        command_info.ssl_vision_id, kRobotRadius,
        obstacle_flag_lookup_table_[i], current_info.position.translation,
        current_velocity, command_info.acceleration_command.translation);
    our_robot_command_info.Verify();
    our_robots.InsertBack(our_robot_command_info);
  }

  const TrajectoryArray our_trajectories =
      ComputeTrajectoryCheckpointsArray(our_robots, our_motion_model);

  InitialCommandInfoArray their_robots;
  for (const auto& opposing_robot : world_state_.GetTheirRobots()) {
    if (opposing_robot.confidence <= 0) {
      continue;
    }
    const InitialCommandInfo their_robot_command_info(
        opposing_robot.ssl_vision_id, kRobotRadius,
        obstacle::ObstacleFlag::GetFull(), opposing_robot.position.translation,
        opposing_robot.velocity.translation, {0, 0});
    their_robot_command_info.Verify();
    their_robots.InsertBack(their_robot_command_info);
  }
  const TrajectoryArray their_trajectories =
      ComputeTrajectoryCheckpointsArray(their_robots, their_motion_model);
  return {our_trajectories, their_trajectories};
}

std::tuple<Position, Velocity> DSS2::ComputeEndOfControlInfo(
    const Position& current_position, const Velocity& current_velocity,
    const Acceleration& commanded_acceleration,
    const motion::MotionModel& motion_model, const Time control_period) const {
  const Position end_of_control_position = CalculateEndPositionWithAccel(
      current_position, current_velocity, commanded_acceleration,
      control_period, motion_model);
  const Velocity end_of_control_velocity = CalculateEndVelocityWithAccel(
      current_velocity, commanded_acceleration, control_period, motion_model);

  std::tuple<Position, Velocity> tuple(end_of_control_position,
                                       end_of_control_velocity);
  return tuple;
}

std::tuple<Position, Acceleration, Time> DSS2::ComputeStoppedInfo(
    const Position& end_of_control_position,
    const Velocity& end_of_control_velocity, const Time end_of_control_time,
    const motion::MotionModel& motion_model) const {
  const Velocity normed_velocity = GetNormalizedOrZero(end_of_control_velocity);
  // TODO(kvedder): Remove this artificial limit on the acceleration after cause
  // of lack of deceleration is found.
  const Acceleration deceleration = CapAcceleration(
      normed_velocity * -motion_model.a_max * 0.8, motion_model);

  const Time time_to_rest =
      CalculateTimeToRest(end_of_control_velocity, motion_model);
  const Time halted_time = end_of_control_time + time_to_rest;

  const Position rest_position = CalculateEndPositionWithAccel(
      end_of_control_position, end_of_control_velocity, deceleration,
      time_to_rest, motion_model);

  const std::tuple<Position, Acceleration, Time> result(
      rest_position, deceleration, halted_time);
  return result;
}

DSS2::TrajectoryCheckpoints DSS2::ComputeTrajectoryCheckpoints(
    const InitialCommandInfo& initial_command_info,
    const motion::MotionModel& motion_model) const {
  const SSLVisionId& ssl_vision_id = initial_command_info.ssl_vision_id;
  const float& radius = initial_command_info.radius;
  const Position& initial_position = initial_command_info.initial_position;
  const Position& initial_velocity = initial_command_info.initial_velocity;
  const Acceleration& commanded_acceleration =
      initial_command_info.commanded_acceleration;

  TrajectoryCheckpoints tc;
  tc.ssl_vision_id = ssl_vision_id;
  tc.radius = radius;
  tc.dss_obstacles = initial_command_info.dss_obstacles;

  tc.initial_position = initial_position;
  tc.initial_velocity = initial_velocity;
  tc.initial_commanded_acceleration = commanded_acceleration;

  const auto end_control_info = ComputeEndOfControlInfo(
      initial_position, initial_velocity, commanded_acceleration, motion_model,
      kDSSCommandExecutionPeriod);

  tc.end_control_position = std::get<0>(end_control_info);
  tc.end_control_velocity = std::get<1>(end_control_info);
  tc.end_control_time = kDSSCommandExecutionPeriod;

  const auto stopped_info =
      ComputeStoppedInfo(tc.end_control_position, tc.end_control_velocity,
                         kDSSCommandExecutionPeriod, motion_model);

  tc.halted_position = std::get<0>(stopped_info);
  tc.halting_acceleration = std::get<1>(stopped_info);
  tc.halted_time = std::get<2>(stopped_info);

  tc.Verify();

  return tc;
}

DSS2::TrajectoryArray DSS2::ComputeTrajectoryCheckpointsArray(
    const InitialCommandInfoArray& robots,
    const motion::MotionModel& motion_model) const {
  TrajectoryArray checkpoints_array;
  for (const auto& robot_info_tuple : robots) {
    const DSS2::TrajectoryCheckpoints trajectory_checkpoints =
        ComputeTrajectoryCheckpoints(robot_info_tuple, motion_model);
    checkpoints_array.InsertBack(trajectory_checkpoints);
  }
  return checkpoints_array;
}

DSS2::AlternateAccelerationsArray DSS2::GenerateAlternateAccelerations(
    const Acceleration& commanded_acceleration,
    const motion::MotionModel& motion_model) {
  AlternateAccelerationsArray alternate_commands =
      array_util::MakeArray<kNumNewVelocities>(
          std::make_pair(commanded_acceleration, 0.0f));

  for (size_t i = 0; i < alternate_commands.size(); ++i) {
    const float min_x =
        (i < alternate_commands.size() / 2)
            ? -motion_model.a_max / 4 + commanded_acceleration.x()
            : -motion_model.a_max;
    const float max_x =
        (i < alternate_commands.size() / 2)
            ? motion_model.a_max / 4 + commanded_acceleration.x()
            : motion_model.a_max;
    const float min_y =
        (i < alternate_commands.size() / 2)
            ? -motion_model.a_max / 4 + commanded_acceleration.y()
            : -motion_model.a_max;
    const float max_y =
        (i < alternate_commands.size() / 2)
            ? motion_model.a_max / 4 + commanded_acceleration.y()
            : motion_model.a_max;
    const float rand_x = random.UniformRandom(min_x, max_x);
    const float rand_y = random.UniformRandom(min_y, max_y);
    const Acceleration proposed_acceleration =
        CapAcceleration({rand_x, rand_y}, motion_model);
    alternate_commands[i] = {
        proposed_acceleration,
        (proposed_acceleration - commanded_acceleration).squaredNorm()};
  }

  // Sort it so that the lower cost plans are first. This way, we can do a
  // single linear sweep of the array, halting when we arrive at one that's
  // collision free.
  std::sort(alternate_commands.begin(), alternate_commands.end(),
            [](const std::pair<Acceleration, float>& a,
               const std::pair<Acceleration, float>& b) -> bool {
              return a.second < b.second;
            });

  return alternate_commands;
}

bool DSS2::AttemptAlternateCommands(
    DSS2::TrajectoryCheckpoints* trajectory_in_question,
    const tactics::TacticIndex& trajectory_in_question_tactic,
    const DSS2::TrajectoryArray& our_trajectories,
    const MotionModel& our_motion_model,
    const DSS2::TrajectoryArray& their_trajectories,
    const MotionModel& their_motion_model,
    const obstacle::ObstacleFlag& obstacle_flag, logger::Logger* logger) {
  static constexpr bool kDebug = false;
  const AlternateAccelerationsArray alternate_commands =
      GenerateAlternateAccelerations(
          trajectory_in_question->initial_commanded_acceleration,
          our_motion_model);
  for (const std::pair<Acceleration, float>& c : alternate_commands) {
    const Acceleration& alternate_command = c.first;
    const TrajectoryCheckpoints alternate_trajectory =
        MakeNewAccelerationTrajectory(*trajectory_in_question, our_motion_model,
                                      alternate_command);
    if (kDebug) {
      logger->AddCircle(alternate_trajectory.halted_position, kRobotRadius, 1,
                        1, 1, 0.2);
    }
    if (!TrajectoryCollidesWithWorld(
            alternate_trajectory, trajectory_in_question_tactic,
            our_trajectories, our_motion_model, their_trajectories,
            their_motion_model, obstacle_flag, logger)) {
      *trajectory_in_question = alternate_trajectory;
      return true;
    }
  }
  return false;
}

void DSS2::ResetObstacleFlagTable() {
  obstacle_flag_lookup_table_.fill(obstacle::ObstacleFlag::GetFull());
}

void DSS2::SetObstacleFlag(const OurRobotIndex our_index,
                           const obstacle::ObstacleFlag& flags) {
  NP_CHECK(our_index < kMaxTeamRobots);
  obstacle_flag_lookup_table_[our_index] = flags;
}

}  // namespace safety
