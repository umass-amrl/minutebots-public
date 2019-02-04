// Copyright 2017 - 2018 kvedder@umass.edu
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

#ifndef SRC_TACTICS_EIGHT_GRID_NAVIGATION_H_
#define SRC_TACTICS_EIGHT_GRID_NAVIGATION_H_

#include <fstream>
#include <memory>
#include <vector>

#include "constants/constants.h"
#include "navigation/production/production_eight_grid.h"
#include "obstacles/safety_margin.h"
#include "state/shared_state.h"
#include "tactics/tactic.h"

namespace tactics {
class EightGridNavigation : public Tactic {
 public:
  EightGridNavigation(const state::WorldState& world_state,
                      TacticArray* tactic_list,
                      state::SharedState* shared_state,
                      OurRobotIndex our_robot_index_,
                      state::SoccerState* soccer_state, const bool is_goalie);

  ~EightGridNavigation();

  const char* Name() const override { return "eight_grid_navigation"; }
  void Init() override;
  void Run() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void Reset() override;
  void SetObstacles(const obstacle::ObstacleFlag& obstacles);
  void SetLocationThreshold(const float location_threshold);
  void SetLinearVelocityThreshold(const float linear_velocity_threshold);
  void SetAngularThreshold(const float angular_threshold);
  void SetAngularVelocityThreshold(const float angular_velocity_threshold);
  bool IsComplete() override;

 private:
  void MoveRobotsToOurHalfDuringKickoffHack(
      const Eigen::Vector2f& current_position);

  bool FixedStartedInCollision(logger::Logger* robot_logger,
                               const Eigen::Vector2f& current_position,
                               Tactic* ntoc_controller) const;

  void HandleNoPathFound(logger::Logger* robot_logger,
                         const Eigen::Vector2f& current_position,
                         Tactic* ntoc_controller) const;

  bool IsStraightPathCollisionFree(const Eigen::Vector2f& start,
                                   const Eigen::Vector2f& goal) const;

  Eigen::Vector2f PickNTOCWaypoint(
      logger::Logger* robot_logger, const std::vector<Eigen::Vector2f>& path,
      const Eigen::Vector2f& current_position) const;

  bool PlanCollisionFree(const std::vector<Eigen::Vector2f>& plan) const;

  std::vector<Eigen::Vector2f> DecideToUseOldOrNewPlan(
      logger::Logger* robot_logger,
      const std::vector<Eigen::Vector2f>& new_plan);

  // Returns true if it had to move the goal, false otherwise
  bool SanitizeGoal(logger::Logger* robot_logger);

  void VerifyGoalCollisionFree() const;

  void DrawObstacles(logger::Logger* robot_logger);

  void DrawPath(logger::Logger* robot_logger,
                const std::vector<Eigen::Vector2f>& plan_path);

  bool IsWithinTranslationThreshold();
  bool IsWithinRotationThreshold();
  bool IsWithinCompletedThresholds();

  navigation::production::eight_grid::ProductionEightGrid eight_grid_;
  pose_2d::Pose2Df goal_;
  obstacle::ObstacleFlag obstacles_;
  obstacle::SafetyMargin margin_;

  std::vector<Eigen::Vector2f> previous_plan_;
  float previous_plan_cost_;
  std::ofstream timing_file_;
  static constexpr bool kEightGridDumpTimings = false;
  float location_threshold_squared_;
  float linear_velocity_threshold_squared_;
  float angular_threshold_;
  float angular_velocity_threshold_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_EIGHT_GRID_NAVIGATION_H_
