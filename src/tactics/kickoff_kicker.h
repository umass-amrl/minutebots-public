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

#include <memory>
#include <string>
#include <vector>

#ifndef SRC_TACTICS_KICKOFF_KICKER_H_
#define SRC_TACTICS_KICKOFF_KICKER_H_

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"

namespace tactics {

class KickoffKicker : public StateMachineTactic {
 public:
  KickoffKicker(const string& machine_name,
                const state::WorldState& world_state, TacticArray* tactic_list,
                state::SharedState* shared_state, OurRobotIndex our_robot_index,
                state::SoccerState* soccer_state);

  const char* Name() const override { return "kickoff_kicker"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;

  // True when:
  // 1) Within angular error margin from aim &&
  // 2) Within desired radial distance &&
  // 3) at relative rest with respect to the ball &&
  // 4) in alignment with the ball &&
  // 5) is not rotating rapidly &&
  // 6) is within a relative rest threshold along the y-axis &&
  // 7) if kicking has already started
  bool ShouldKick(logger::Logger* the_logger, const float target_angle,
                  const Vector2f current_ball_pose,
                  const Vector2f current_ball_velocity,
                  const pose_2d::Pose2Df current_robot_pose,
                  const pose_2d::Pose2Df current_robot_velocity,
                  const bool is_currenlty_kicking, const bool has_timed_out,
                  const bool debug);

  float target_angle_;
  OurRobotIndex target_robot_;
  Eigen::Vector2f target_position_;

 private:
  void Transition() override;

  // Behavior Functions Used by the states
  void Preparation();

  void Setup();

  void Kick();

  void PostKick();

  bool is_complete_;
  OurRobotIndex last_target_;
  bool first_;

  // States
  State preparation_;
  State setup_;
  State kick_;
  State post_kick_;
  // THRESHOLDS TO TUNE
  // Maximal angular difference
  // between robot and goal to kick
  // (rad)
  RepairableParam thresholds_angle_;
  // Maxmimum distance between robot and ball to kick
  // (mm)
  RepairableParam thresholds_distance_;
  // Maxmimum relative y velocity between robot and ball
  // to kick
  // (mm/s)
  RepairableParam thresholds_y_prime_vel_;
  // Maxmimum total relative velocity to kick
  //
  RepairableParam thresholds_relative_vel_;
  // Maxmimum y offset from ball to kick
  // (mm)
  RepairableParam thresholds_align_;
  // Maxmimum angular velocity to kick
  // (rad/s)
  RepairableParam thresholds_angular_vel_;
  // Maxmimum number of timesteps to kick for
  RepairableParam thresholds_kick_timeout_;
  // Velocity threshold for intercept
  // (mm/s)
  RepairableParam thresholds_ball_velocity_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_KICKOFF_KICKER_H_
