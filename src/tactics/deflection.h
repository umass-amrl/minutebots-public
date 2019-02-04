// Copyright 2018 tszkeiserena@umass.edu jaholtz@cs.umass.edu
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
#include <tuple>
#include <vector>

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"

#ifndef SRC_TACTICS_DEFLECTION_H_
#define SRC_TACTICS_DEFLECTION_H_

namespace tactics {

class Deflection : public StateMachineTactic {
 public:
  Deflection(const string& machine_name, const state::WorldState& world_state,
             TacticArray* tactic_list, state::SharedState* shared_state,
             OurRobotIndex our_robot_index, state::SoccerState* soccer_state);

  const char* Name() const override { return "deflection"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  float GetCost() override;
  bool IsComplete() override;
  bool BadDeflection();
  bool BadTiming();
  bool SettingUp();
  const pose_2d::Pose2Df GetGoal() override;
  bool IsKick();

 private:
  void Transition() override;

  void Start();
  void Setup();
  void Wait();
  void Kick();
  void Finish();

  float GetRobotKickTime(float robot_speed, float current_distance);
  float GetBallArrivalTime(float ball_velocity, float current_distance);
  Vector2f ShiftInterceptionPoint(Vector2f current_point, Vector2f dir_vector,
                                  float offset);
  Vector2f GetInterceptionPoint(Vector2f robot_current, Vector2f robot_goal,
                                Vector2f ball_current, Vector2f ball_next,
                                float tolerance);

  State start_;
  State setup_;
  State wait_;
  State kick_;
  State finish_;

  // Robot makes no attempt to deflect if ball-path angle larger than this
  // threshold
  RepairableParam ball_angle_threshold_;
  // Robot makes no attempt to deflect if ball is below this velocity
  RepairableParam ball_velocity_threshold_;
  // Beta in the CMU 2008 TDP
  RepairableParam dribber_dampening_factor_;
  // Gamma in the CMU 2008 TDP
  RepairableParam ball_velocity_scale_;
  // Precision of the angle when repositioning
  RepairableParam angle_precision_threshold_;
  // Distance of the waiting robot from the interception point
  RepairableParam robot_wait_threshold_y_max_;
  // Distance tolerance
  RepairableParam robot_wait_threshold_tolerance_;
  // Maximum left-right tolerance
  RepairableParam robot_wait_threshold_x_;
  // Max time difference between ball/robot
  RepairableParam time_diff_threshold_;
  // Threshold for percent of velocity along robot x-axis
  // for a successful kick.
  RepairableParam kicked_x_threshold_;
  // Threshold for determing if kicked
  RepairableParam kicked_velocity_threshold_;
  RepairableParam thresholds_kick_timeout_;
  RepairableParam thresholds_kick_percent_;
  RepairableParam thresholds_kick_speed_;
  RepairableParam thresholds_follow_through_;
  RepairableParam thresholds_distance_;
  RepairableParam thresholds_setup_distance_;
  RepairableParam thresholds_deflection_angle_;

  // Point of deflection
  Vector2f interception_point_;
  // Point of stopping
  Vector2f interception_point_actual_;
  // Target net angle (angle of the ball)
  float interception_angle_goal_;
  // Actual angle of the robot to achieve target angle
  float interception_angle_actual_;
  // Distance from kicking
  float waiting_buffer_;
  // Large angle fofset
  // float large_angle_offset_base_;
  // kick count
  int kick_count_;
  // in simulation?
  bool simulation_;
  // Tolerance for colinear check
  float colinear_tolerance_;
  bool complete_;
  bool passing_;
  OurRobotIndex last_target_;

  const float kWaitingDistance = kRobotRadius + kBallRadius;
  static const bool kDebug = false;
};
}  // namespace tactics

#endif  // SRC_TACTICS_DEFLECTION_H_
