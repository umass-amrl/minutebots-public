// Copyright 2017-2018 slane@cs.umass.edu, kvedder@umass.edu,
// jaholtz@cs.umass.edu
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

#ifndef SRC_STATE_WORLD_STATE_H_
#define SRC_STATE_WORLD_STATE_H_

#include <array>
#include <map>
#include <memory>
#include <vector>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "datastructures/bounded_queue.h"
#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "obstacles/ball_obstacle.h"
#include "obstacles/obstacle.h"
#include "obstacles/rectangle_obstacle.h"
#include "obstacles/robot_obstacle.h"
#include "state/position_velocity_state.h"
#include "state/shared_state.h"
#include "state/team.h"
#include "state/world_ball.h"
#include "state/world_robot.h"

namespace motion_model {
class DefaultMotionModel;
}  // namespace motion_model

namespace logger {
class Logger;
}  // namespace logger

namespace state {

// This class holds data regarding the physical state of the world.
class WorldState {
 public:
  WorldState() = delete;
  // Uses the position_velocity_state reference as the stored memory. Assumes
  // that it will be automatically updated when needed.
  WorldState(state::PositionVelocityState* position_velocity_state,
             const team::Team team, const bool simulating = false);
  WorldState(const WorldState& other) = delete;
  WorldState(WorldState&& other) = delete;
  ~WorldState();
  WorldState& operator=(const WorldState& other) = delete;
  WorldState& operator=(WorldState&& other) = delete;

  // Updates internal state based on time and updates to the
  // position_velocity_state.
  void UpdateState(logger::Logger* logger);

  const int GetOurRobotIndex(SSLVisionId our_vision_id) const;
  const int GetTheirRobotIndex(SSLVisionId their_vision_id) const;

  const state::PositionVelocityState::RobotPositionVelocity&
  GetOurRobotPosition(OurRobotIndex our_robot_index) const;
  const state::PositionVelocityState::RobotPositionVelocity&
  GetTheirRobotPosition(TheirRobotIndex their_robot_index) const;

  const datastructures::DenseArray<PositionVelocityState::RobotPositionVelocity,
                                   kMaxTeamRobots>&
  GetOurRobots() const;

  const datastructures::DenseArray<PositionVelocityState::RobotPositionVelocity,
                                   kMaxTeamRobots>&
  GetTheirRobots() const;

  const std::array<std::unique_ptr<obstacle::Obstacle>, kNumObstacles>&
  GetAllObstacles() const;

  const state::PositionVelocityState::BallPositionVelocity& GetBallPosition()
      const;

  const state::PositionVelocityState::BallPositionVelocity&
  GetLastBallPosition() const;

  const team::Team& GetOurTeam() const;

  const state::PositionVelocityState& GetPositionVelocityState() const;

  double GetWorldTime() const;

  double GetLastWorldTime() const;

  const size_t GetNumOurRobots() const;
  const size_t GetNumTheirRobots() const;

  void UpdateOurObservation(const OurRobotIndex index,
                            const pose_2d::Pose2Df& observation,
                            const float confidence,
                            const double observation_time);

  void UpdateLastState(const state::PositionVelocityState& state);

  void AddSharedState(SharedState shared_state);

  // Returns true if the robot with the particular vision ID can chip kick
  bool CanChip(const SSLVisionId robot_id) const;

  // Calling this variable directly is DEPRECATED! Use GetWorldTime() instead!
  double world_time_;

  double last_world_time_;

  // Static instance of all obstacles. Do NOT access this outside of the
  // executor thread. Do NOT modify this vector directly.
  //   static std::vector<std::unique_ptr<obstacle::Obstacle>> static_obstacles;
  static std::array<std::unique_ptr<obstacle::Obstacle>, kNumObstacles>
      master_obstacle_buffer;

  const motion_model::DefaultMotionModel& GetOurMotionModel(
      OurRobotIndex index) const;

  const motion_model::DefaultMotionModel& GetTheirMotionModel(
      TheirRobotIndex index) const;

  // Update with the most recent processing time
  const void UpdateProcessingTime(double new_time);

 private:
  void ForwardPredictRobot(SSLVisionId ssl_vision_id, bool is_our_team,
                           double start_timestamp, double end_timestamp,
                           const pose_2d::Pose2Df& current_position,
                           const pose_2d::Pose2Df& current_velocity,
                           const motion_model::DefaultMotionModel& model,
                           pose_2d::Pose2Df* next_position,
                           pose_2d::Pose2Df* next_velocity,
                           logger::Logger* logger) const;

  void ClearPastCommands(double timestamp);

  state::PositionVelocityState* position_velocity_state_;
  state::PositionVelocityState last_pv_state_;
  const team::Team team_;
  const bool simulating_;

  // Note: These have to be vectors. The DefaultMotionModel class needs to be
  // forward declared and a std::array requires a complete type
  std::vector<motion_model::DefaultMotionModel> our_motion_models_;

  std::vector<motion_model::DefaultMotionModel> their_motion_models_;

  // Current shared state here means the one right before the first element of
  // past_shared_states_
  SharedState current_shared_state_;
  std::vector<SharedState> past_shared_states_;

  // Processing time averaging structures
  // kNumTimes is the size of the running average
  // kExpectedAverage_ is the expected average processing time, this gets added
  // to the queue at the begining but will get overwritten
  static constexpr unsigned int kNumTimes_ = 10;
  static constexpr double kExpectedAverage_ = 0.0;
  double avg_processing_time_;
  double time_queue_sum_;
  datastructures::BoundedQueue<double> processing_time_queue_;

  static const bool kLogPredictions_ = false;
};
}  // namespace state

#endif  // SRC_STATE_WORLD_STATE_H_
