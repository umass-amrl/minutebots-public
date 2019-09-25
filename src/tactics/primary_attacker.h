// Copyright 2018 - 2019 jaholtz@cs.umass.edu
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

#ifndef SRC_TACTICS_PRIMARY_ATTACKER_H_
#define SRC_TACTICS_PRIMARY_ATTACKER_H_

#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "state/shared_state.h"
#include "tactics/state_machine_tactic.h"
#include "third_party/json.hpp"

namespace tactics {

class PrimaryAttacker : public StateMachineTactic {
 public:
  PrimaryAttacker(const string& machine_name,
                  const state::WorldState& world_state,
                  TacticArray* tactic_list, state::SharedState* shared_state,
                  OurRobotIndex our_robot_index,
                  state::SoccerState* soccer_state);

  const char* Name() const override { return "primary_attacker"; }
  void Init() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  bool IsComplete() override;
  float GetCost() override;
  const Eigen::Vector2f GetNextInterceptPoint();
  const pose_2d::Pose2Df GetGoal() override;
  void SetThresholds(const float& angle, const float& distance,
                     const float& align, const float& y_prime_vel,
                     const float& angular_vel, const float& relative_vel,
                     const float& kick_timeout, const float& catch_velocity,
                     const float& could_score_vel, const float& intercept_vel,
                     const float& catch_trajectory, const float& toward_robot);
  void LoadConfigFile();
  void SetValue(nlohmann::json config_json, RepairableParam* param);
  void SetState(nlohmann::json config_json);
  void WriteTrainingData(const State& last_state);
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
  OurRobotIndex last_target_;

 private:
  void GetTarget(const Eigen::Vector2f& source, float* angle,
                 OurRobotIndex* target_robot);
  bool ShouldGoToBall(const Eigen::Vector2f& ball,
                      const Eigen::Vector2f& robot);

  bool ShouldSTOXPivot(const Vector2f current_ball_pose,
                       const Vector2f current_ball_velocity,
                       const pose_2d::Pose2Df current_robot_pose,
                       const pose_2d::Pose2Df current_robot_velocity) const;

  // True when:
  // 1) Ball is moving sufficiently fast &&
  // 2) Ball trajectory is not likely to score &&
  // 3) Ball trajectory is predominantly in the direction of the robot
  bool ShouldCatch(logger::Logger* the_logger, const float target_angle,
                   const Vector2f current_ball_pose,
                   const Vector2f current_ball_velocity,
                   const pose_2d::Pose2Df current_robot_pose,
                   const pose_2d::Pose2Df current_robot_velocity,
                   const bool is_currenlty_kicking, const bool has_timed_out,
                   const bool debug);

  bool CouldBallScore(logger::Logger* the_logger, const float target_angle,
                      const Vector2f current_ball_pose,
                      const Vector2f current_ball_velocity,
                      const pose_2d::Pose2Df current_robot_pose,
                      const pose_2d::Pose2Df current_robot_velocity,
                      const bool is_currenlty_kicking, const bool has_timed_out,
                      const bool debug);

  bool DoesPathCrossBallRay();

  // True when:
  // 1) ball is moving sufficiently fast &&
  // 2) intercept position is within the field &&
  // 3) intercept position is not in the opposing defense area &&
  // 4) the intercept path is collision free &&
  // 5) the intercept path does not cross the balls path
  bool ShouldIntercept();

  bool ShouldNavigateToCatch();

  // True when ball is going sufficiently slow.
  bool ShouldGoToBall();

  void Transition() override;

  void GetSolution(Vector2f current_robot_trans,
                   const Vector2f& current_velocity_world,
                   const Vector2f& current_ball_pose,
                   const Vector2f& current_ball_velocity);

  // Behavior Functions Used by the states
  // Navigates to a kick position with respect to the ball and target.
  void Start();

  void Catch();

  void Intercept();

  void Kick();

  void PostKick();

  void NavigateCatch();

  void NavigateIntercept();

  void Receive();

  void STOXPivot();

  void ManualTransition();
  void NetTransition();

  SolutionParameters intercept_solution_;
  SolutionParameters catch_solution_;
  SolutionParameters kick_solution_;
  Vector2f robot_interception_point_;
  Vector2f last_robot_interception_point_;
  Vector2f robot_interception_vel_;
  Vector2f ball_intercept_point_;
  Vector2f ball_interception_vel_;

  Vector2f robot_catch_point_;
  Vector2f robot_catch_vel_;
  Vector2f ball_catch_point_;
  Vector2f ball_catch_vel_;
  // expected time addition from using NTOC over TSOCS
  const float kInterceptionNavigationTimeAdjustment = 1.2;
  const float kCatchNavigationTimeAdjustment = 1.2;

  bool set_kick_goal;
  bool chip_;
  float chip_distance_;
  bool is_complete_;
  bool should_pass_;
  bool intercepting_;
  int aim_count_;
  const int kMaxAimIterations;

  // States
  State start_;
  State catch_;
  State intercept_;
  State kick_;
  State post_kick_;
  State navigate_to_catch_;
  State navigate_to_intercept_;
  State receive_;
  State stox_pivot_;
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
  // Velocity threshold for catch
  // (mm/s)
  RepairableParam thresholds_lower_catch_velocity_;
  // velocity threshold for a possible goal
  // (mm/s)
  RepairableParam thresholds_could_score_speed_;
  // Velocity threshold for intercept
  // (mm/s)
  RepairableParam thresholds_ball_velocity_;
  // Minimum portion of ball velocity towards robot for catch
  // (percentage)
  RepairableParam thresholds_catch_radius_;
  // Minimum portion of the velocity in the kick direction for successful kick
  RepairableParam thresholds_kick_percent_;
  RepairableParam thresholds_kick_speed_;
  RepairableParam thresholds_follow_through_;
  // END THRESHOLDS
  int kick_count_;
  int prep_count_;
  float last_target_score_;
  OurRobotIndex last_target_robot_;
  Vector2f last_target_position_;

  const float kRotationRadiusMargin_ = 40.0;
  const int kPrepCountStart_ = 60;
  const float kBadSolutionthreshold_ = 5.0;
  const float kPassChangeThreshold_ = .005;
  const bool pass_only_ = false;
  const float kPassReduction = 0.1;
  const float kNavigatePenalty = 1.5;
  const float kSetPassDistance = 5 * kRobotRadius;
  //   const float kInterceptReduction = .98;
  const float kCatchReduction = 1;
  std::ofstream data_file;
};
}  // namespace tactics

#endif  // SRC_TACTICS_PRIMARY_ATTACKER_H_
