// Copyright 2018 jaholtz@cs.umass.edu
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

#ifndef SRC_TACTICS_COERCIVE_ATTACKER_H_
#define SRC_TACTICS_COERCIVE_ATTACKER_H_

#include <memory>
#include <string>
#include <vector>

#include "evaluators/offense_evaluation.h"
#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"
#include "util/random.h"
#include "zone/zone.h"

namespace tactics {
class CoerciveAttacker : public StateMachineTactic {
 public:
  CoerciveAttacker(const string& machine_name,
                   const state::WorldState& world_state,
                   TacticArray* tactic_list, state::SharedState* shared_state,
                   OurRobotIndex our_robot_index,
                   state::SoccerState* soccer_state);

  const char* Name() const override { return "coercive_attacker"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;

 private:
  // Given a specific pose on the field, returns it's cost
  // with respect to this robot.
  float GetPoseCost(const Vector2f& position,
                    OurRobotIndex support_attacker_index,
                    pose_2d::Pose2Df* proposed_pose,
                    bool use_input_pass_clearance,
                    float calculated_pass_clearance);

  float GetFacing();
  Vector2f GetStoppingPose();
  float TimeToReceive(const Eigen::Vector2f stopping_pose);
  bool ShouldNavigate();

  pose_2d::Pose2Df FindBestPosition();

  // Sets the attacker_index_ to the index of the primary offensive robot.
  void GetPrimaryAttacker();

  void Navigate();
  void Receive();
  void Wait();

  State navigate_;
  State receive_;
  State wait_;
  void Transition() override;

  RepairableParam thresholds_setup_time_;

  bool is_complete_;
  OurRobotIndex last_target_;
  OurRobotIndex attacker_index_;
  float last_cost_;
  pose_2d::Pose2Df last_goal_;
  pose_2d::Pose2Df evaluated_target_;
  pose_2d::Pose2Df wait_target_;
  bool chip_;

  // Used to determine whether the support attacker should look at the goal
  // or the ball
  const float min_goal_open_angle = (1.0 / 180.0) * M_PI;

  // The maximum passing clearance angles used for saturating the angle
  const float passing_clearance_max = (20.0 / 180.0) * M_PI;

  // Used to prevent rapid switching when there exists no open angle
  // towards the goal.
  const float goal_open_angle_bias = (1.0 / 180.0) * M_PI;

  // Used as a search window size around the current robot position
  const float kSearchRadius = 2 * kRobotRadius;

  // Number of rings of kSearchRadius to search away from the robot.
  const int kSearchRings = 25;

  // Used to prevent the support attackers from camping close
  // to the primary offensive role
  const float kAttackerPadding = 15.0 * kRobotRadius;

  const float kRobotPadding = 15.0 * kRobotRadius;

  const bool kDebug = false;
  // Minimum cost difference in order to switch goal
  const float cost_diff_threshold = 0.001;

  const float kTooFastThreshold = 4000;
};
}  // namespace tactics

#endif  // SRC_TACTICS_COERCIVE_ATTACKER_H_
