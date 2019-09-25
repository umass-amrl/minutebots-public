// Copyright 2018 - 2019 kvedder@umass.edu
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

#include <cmath>
#include <fstream>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#include "math/poses_2d.h"
#include "motion_control/motion_model.h"
#include "safety/dss_helpers.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "tactics/tactic_index.h"
#include "util/random.h"

#ifndef SRC_SAFETY_DSS2_H_
#define SRC_SAFETY_DSS2_H_

namespace safety {

class DSS2 {
  // Number of new velocities to generate.
  static constexpr unsigned int kNumNewVelocities = 100;

 public:
  DSS2() = delete;
  DSS2(const state::WorldState& world_state, state::SoccerState* soccer_state);
  ~DSS2() = default;

  void MakeSafe(const motion::MotionModel& our_motion_model,
                const motion::MotionModel& their_motion_model,
                logger::Logger* logger);

  struct TrajectoryCheckpoints {
    SSLVisionId ssl_vision_id;
    float radius;
    obstacle::ObstacleFlag dss_obstacles;

    Position initial_position;
    Velocity initial_velocity;
    Acceleration initial_commanded_acceleration;

    Position end_control_position;
    Velocity end_control_velocity;
    Time end_control_time;

    Position halted_position;
    Acceleration halting_acceleration;
    Time halted_time;
    TrajectoryCheckpoints()
        : ssl_vision_id(0),
          radius(0),
          dss_obstacles(),
          initial_position(0, 0),
          initial_velocity(0, 0),
          initial_commanded_acceleration(0, 0),
          end_control_position(0, 0),
          end_control_velocity(0, 0),
          end_control_time(0),
          halted_position(0, 0),
          halting_acceleration(0, 0),
          halted_time(0) {}

    bool operator==(const TrajectoryCheckpoints& other) const {
      return ssl_vision_id == other.ssl_vision_id && radius == other.radius &&
             initial_position == other.initial_position &&
             initial_velocity == other.initial_velocity &&
             initial_commanded_acceleration ==
                 other.initial_commanded_acceleration &&
             end_control_position == other.end_control_position &&
             end_control_velocity == other.end_control_velocity &&
             end_control_time == other.end_control_time &&
             halted_position == other.halted_position &&
             halting_acceleration == other.halting_acceleration &&
             halted_time == other.halted_time;
    }

    void Verify() const {
      NP_CHECK_MSG(end_control_time <= halted_time,
                   "End control time: " << end_control_time << " Halted time: "
                                        << halted_time);
      NP_CHECK_MSG(end_control_time >= 0,
                   "End control time: " << end_control_time);
      NP_CHECK_MSG(halted_time >= 0, "Halted time: " << halted_time);
    }
  };

  struct InitialCommandInfo {
    SSLVisionId ssl_vision_id;
    float radius;
    obstacle::ObstacleFlag dss_obstacles;
    Position initial_position;
    Velocity initial_velocity;
    Acceleration commanded_acceleration;

    InitialCommandInfo()
        : ssl_vision_id(0),
          radius(0),
          dss_obstacles(),
          initial_position(0, 0),
          initial_velocity(0, 0),
          commanded_acceleration(0, 0) {}
    InitialCommandInfo(const SSLVisionId& ssl_vision_id,
                       const float& radius,
                       const obstacle::ObstacleFlag dss_obstacles,
                       const Position& initial_position,
                       const Velocity& initial_velocity,
                       const Acceleration& commanded_acceleration)
        : ssl_vision_id(ssl_vision_id),
          radius(radius),
          dss_obstacles(dss_obstacles),
          initial_position(initial_position),
          initial_velocity(initial_velocity),
          commanded_acceleration(commanded_acceleration) {
      if (commanded_acceleration.squaredNorm() > Sq(kMaxRobotAcceleration)) {
        this->commanded_acceleration =
            commanded_acceleration.normalized() * kMaxRobotAcceleration;
      }
    }

    void Verify() const {
      NP_CHECK(!std::isnan(ssl_vision_id));
      NP_CHECK(!std::isnan(radius));
      NP_CHECK(!std::isnan(initial_position.x()));
      NP_CHECK(!std::isnan(initial_position.y()));
      NP_CHECK(!std::isnan(initial_velocity.x()));
      NP_CHECK(!std::isnan(initial_velocity.y()));
      NP_CHECK(!std::isnan(commanded_acceleration.x()));
      NP_CHECK(!std::isnan(commanded_acceleration.y()));
    }
  };

  using InitialCommandInfoArray =
      datastructures::DenseArray<InitialCommandInfo, kMaxTeamRobots>;
  using TrajectoryArray =
      datastructures::DenseArray<DSS2::TrajectoryCheckpoints, kMaxTeamRobots>;
  using ObstacleFlagTable = std::array<obstacle::ObstacleFlag, kMaxTeamRobots>;

  static void ResetObstacleFlagTable();

  static void SetObstacleFlag(const OurRobotIndex our_index,
                              const obstacle::ObstacleFlag& flags);

 private:
  void UpdateSharedState(const DSS2::TrajectoryArray& our_trajectories);

  std::pair<TrajectoryCheckpoints, motion::MotionModel> ComputeBallMotion()
      const;

  bool RepairStartInCollision(
      DSS2::TrajectoryCheckpoints* trajectory_in_question,
      const tactics::TacticIndex& trajectory_in_question_tactic,
      const DSS2::TrajectoryArray& our_trajectories,
      const motion::MotionModel& our_motion_model,
      const DSS2::TrajectoryArray& their_trajectories,
      const motion::MotionModel& their_motion_model,
      const TrajectoryCheckpoints& ball_trajectory,
      const obstacle::ObstacleFlag& obstacle_flag,
      logger::Logger* logger) const;

  TrajectoryCheckpoints MakeNewAccelerationTrajectory(
      const TrajectoryCheckpoints& existing_trajectory,
      const motion::MotionModel& motion_model,
      const Acceleration& new_accel) const;

  TrajectoryCheckpoints MakeHaltingTrajectory(
      const TrajectoryCheckpoints& existing_trajectory,
      const motion::MotionModel& motion_model) const;

  bool TrajectoryCollidesWithTrajectoryArray(
      const DSS2::TrajectoryCheckpoints& trajectory,
      const motion::MotionModel& trajectory_motion_model,
      const DSS2::TrajectoryArray& array,
      const motion::MotionModel& array_motion_model,
      logger::Logger* logger,
      const bool use_variable_margin) const;

  std::tuple<Position, Velocity> ComputeEndOfControlInfo(
      const Position& current_position,
      const Velocity& current_velocity,
      const Acceleration& commanded_acceleration,
      const motion::MotionModel& motion_model,
      const Time control_period) const;

  std::tuple<Position, Acceleration, Time> ComputeStoppedInfo(
      const Position& end_of_control_position,
      const Velocity& end_of_control_velocity,
      const Time end_of_control_time,
      const motion::MotionModel& motion_model) const;

  TrajectoryCheckpoints ComputeTrajectoryCheckpoints(
      const InitialCommandInfo& initial_command_info,
      const motion::MotionModel& motion_model) const;

  std::pair<TrajectoryArray, TrajectoryArray> GetOurTheirTrajectories(
      const motion::MotionModel& our_motion_model,
      const motion::MotionModel& their_motion_model,
      logger::Logger* logger) const;

  bool TrajectoryPairCollide(const TrajectoryCheckpoints& t1,
                             const motion::MotionModel& t1_motion_model,
                             const TrajectoryCheckpoints& t2,
                             const motion::MotionModel& t2_motion_model,
                             const float robot_safety_margin,
                             logger::Logger* logger) const;

  bool TrajectorCollideWithObstacle(const obstacle::Obstacle* obstacle,
                                    const TrajectoryCheckpoints& t1,
                                    const motion::MotionModel& t1_motion_model,
                                    const float robot_safety_margin,
                                    logger::Logger* logger) const;

  TrajectoryArray ComputeTrajectoryCheckpointsArray(
      const InitialCommandInfoArray& robots,
      const motion::MotionModel& motion_model) const;

  using AlternateAccelerationsArray =
      std::array<std::pair<Acceleration, float>, kNumNewVelocities>;

  AlternateAccelerationsArray GenerateAlternateAccelerations(
      const Acceleration& commanded_acceleration,
      const motion::MotionModel& motion_model);

  bool TrajectoryCollidesWithWorld(
      const DSS2::TrajectoryCheckpoints& trajectory_in_question,
      const tactics::TacticIndex& trajectory_in_question_tactic,
      const DSS2::TrajectoryArray& our_trajectories,
      const motion::MotionModel& our_motion_model,
      const DSS2::TrajectoryArray& their_trajectories,
      const motion::MotionModel& their_motion_model,
      const obstacle::ObstacleFlag& obstacle_flag,
      logger::Logger* logger) const;

  bool AttemptAlternateCommands(
      DSS2::TrajectoryCheckpoints* trajectory_in_question,
      const tactics::TacticIndex& trajectory_in_question_tactic,
      const DSS2::TrajectoryArray& our_trajectories,
      const motion::MotionModel& our_motion_model,
      const DSS2::TrajectoryArray& their_trajectories,
      const motion::MotionModel& their_motion_model,
      const obstacle::ObstacleFlag& obstacle_flag,
      logger::Logger* logger);

  // Max move out of obstacle velocity in mm/s
  //   const float kObstacleExitVelocity = kMaxRobotVelocity;
  // Max velocity the robot can be commanding before considered trying to move
  // in mm^2/s^2
  //   const float kMovingVelocityStopped = 900;
  util_random::Random random;
  const state::WorldState& world_state_;
  state::SoccerState* soccer_state_;
  static ObstacleFlagTable obstacle_flag_lookup_table_;
  static constexpr bool kDSSDumpTimings = true;
  std::ofstream timing_file_;
};
}  // namespace safety
#endif  // SRC_SAFETY_DSS2_H_
