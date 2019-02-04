// Copyright 2017-2018 slane@cs.umass.edu, kvedder@umass.edu
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

#ifndef SRC_STATE_SOCCER_STATE_H_
#define SRC_STATE_SOCCER_STATE_H_

#include <memory>
#include <vector>
#include <utility>
#include "constants/constants.h"
#include "datastructures/dense_array.h"
#include "eigen3/Eigen/Core"
#include "logging/logger.h"
#include "navigation/production/collision_grid.h"
#include "radio_protocol_wrapper.pb.h"
#include "state/direction.h"
#include "state/referee_state.h"
#include "state/shared_state.h"
#include "state/soccer_robot.h"
#include "state/team.h"
#include "tactics/tactic.h"
#include "tactics/tactic_index.h"

namespace motion_model {
class DefaultMotionModel;
}  // namespace motion_model

namespace state {

// This class holds data regarding the state of the AI.
class SoccerState {
 public:
  explicit SoccerState(const state::WorldState& world_state);
  SoccerState(const state::WorldState& world_state, const team::Team& team);
  SoccerState(const state::WorldState& world_state,
              const team::Team& team,
              const direction::Direction& direction);
  ~SoccerState();

  // Updates the ordering and existence of soccer robots to match that of the
  // world robots. This is to handle the case when our robots disappear/are
  // added to the field; this dynamically handles the robots and ensures that
  // they remain in sync with world state.
  void UpdateExistances();

  const SoccerRobot& GetRobotByOurRobotIndex(OurRobotIndex robot_index) const;
  logger::Logger* GetMutableRobotLoggerByOurRobotIndex(
      OurRobotIndex robot_index);
  SoccerRobot* GetMutableRobot(OurRobotIndex robot_index);

  void SetRobotTacticByRobotIndex(OurRobotIndex robot_index,
                                  tactics::TacticIndex tactic);

  const std::vector<SoccerRobot>& GetAllSoccerRobots() const;
  std::vector<SoccerRobot>* GetAllMutableSoccerRobots();

  const SharedState& GetSharedState() const;
  SharedState* GetMutableSharedState();

  void UpdateNavigationState(logger::Logger* logger);

  void RunAllTactics();

  const team::Team& GetTeam() const;

  const WorldState& GetWorldState();

  const RefereeState& GetRefereeState() const;

  void SetRefereeState(const RefereeState& ref_state);

  bool IsBallKicked();

  bool BallOurHalf();

  bool BallTheirHalf();

  bool BallMidField();

  bool BallOurPossession();

  bool BallTheirPossession();

  bool FreeBall();

  bool IsNormalPlay() const;

  bool IsKickoff() const;

  bool IsOurPenaltyKick() const;

  bool IsTheirPenaltyKick() const;

  bool IsBallMoved() const;

  void UpdateGameState();

  bool BallWasPassed() const;

  SSLVisionId GetReceivingRobot() const;

  void SetBallWasPassed(SSLVisionId receiving_robot_id, double pass_time);

  void UpdateBallWasPassed();

  const pose_2d::Pose2Df GetGoalByRobotIndex(OurRobotIndex robot_index);

  const navigation::production::eight_grid::CollisionGrid&
  GetStaticCollisionGrid() const;
  const navigation::production::eight_grid::CollisionGrid&
  GetDynamicCollisionGrid() const;

  std::array<std::pair<Vector2f, bool>, 8> setup_positions_;
  std::array<float, 8> setup_scores_;
  std::array<Vector2f, 8> aggressive_setup_positions_;
  std::array<float, 8> aggressive_setup_scores_;
  float best_pass_angle_;
  Vector2f best_pass_position_;

  direction::Direction direction_;

 private:
  // The state of each of the robots
  std::vector<SoccerRobot> robots_;

  team::Team our_team_;
  bool normal_play_ = false;
  bool kickoff_ = false;
  bool our_penalty_kick_ = false;
  bool their_penalty_kick_ = false;

  // References to this are given to all tactics, such that when the tactic is
  // executed, this is what gets updated.
  SharedState shared_state_;

  const WorldState& world_state_;
  RefereeState referee_state_;

  navigation::production::eight_grid::CollisionGrid static_collision_grid_;
  navigation::production::eight_grid::CollisionGrid dynamic_collision_grid_;

  bool ball_was_passed_;
  SSLVisionId receiving_robot_id_;
  double pass_time_;

  Eigen::Vector2f ball_start_;
  double ball_start_update_time_;
  std::ofstream eight_grid_timing_file_;
  static constexpr bool kDumpCollisionRebuild = false;
};
}  // namespace state

#endif  // SRC_STATE_SOCCER_STATE_H_
