// Copyright 2017-2018 slane@cs.umass.edu
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

#ifndef SRC_EVALUATORS_DEFENSE_EVALUATION_H_
#define SRC_EVALUATORS_DEFENSE_EVALUATION_H_

#include <functional>
#include <string>
#include <vector>

#include "datastructures/vector_priority_queue.h"
#include "evaluators/offense_evaluation.h"
#include "state/soccer_state.h"
#include "state/world_state.h"

namespace defense {

struct PrimaryThreat {
  PrimaryThreat();
  pose_2d::Pose2Df defender_pose_;
  bool is_blocked_;
  OurRobotIndex blocking_robot_id_;
  bool is_intercept_;
  bool notify_goalie_;
};

struct SecondaryThreat {
  SecondaryThreat();
  // Whether the line from the robot to the goal is blocked
  bool direct_blocked_;

  // Whether the line from the primary threat to the robot is blocked
  bool indirect_blocked_;

  // Time for the ball to reach the robot plus the time for the target to shoot
  float score_;

  // Robot location
  Eigen::Vector2f threat_position_;

  // Where a primary defender should block
  Eigen::Vector2f primary_target_;

  // Where a secondary defender should block
  Eigen::Vector2f secondary_target_;

  // Where a robot should position to disrupt the pass
  Eigen::Vector2f tertiary_target_;

  // The width of the shooting angle
  float open_angle_width_;

  // The opponent robot ID
  SSLVisionId robot_id_;

  // The ID of the robot blocking the line from the opponent to the goal
  SSLVisionId direct_blocking_id_;

  // The ID of the robot blocking the line from the ball to the opponent
  SSLVisionId indirect_blocking_id_;

  friend bool operator<(const SecondaryThreat& first,
                        const SecondaryThreat& second) {
    return first.score_ < second.score_;
  }
};

struct DefenderTargetHelper {
  DefenderTargetHelper();
  bool is_assigned_;
  bool could_be_improved_;
  bool is_direct_;
  float score_;
  float threat_index_;
  SSLVisionId ssl_id_;
  bool is_intercepting_;
};

class DefenseEvaluator {
 public:
  DefenseEvaluator();

  // Helper functions used outside Defense Evaluation
  static bool PointInDefenseArea(const Eigen::Vector2f& point,
                                 float margin,
                                 bool our_defense_area);

  static void SetGoalie(const state::WorldState& world_state);

  // Returns true if the ball is heading towards the specified goal
  // use true for ours and false for theirs
  static bool BallMayEnterGoal(const Eigen::Vector2f ball_position,
                               const Eigen::Vector2f ball_velocity,
                               bool is_ours);

  // Logger Functions
  static void ResetDefenseLogger();
  static const logger::Logger& GetDefenseLogger();

  // Primary Functions
  static void UpdateNumDefenders(const std::vector<std::string>& role_list);

  static void CalculateThreats(const state::WorldState& world_state,
                               const state::SoccerState& soccer_state);

  // Robot Velocity must be in world frame
  static float CalculatePrimaryCost(const Vector2f& robot_position,
                                    const Vector2f& robot_velocity);

  // Robot Velocity must be in world frame
  static float CalculateSecondaryCost(SSLVisionId id,
                                      const Vector2f& robot_position,
                                      const Vector2f& robot_velocity);

  // Robot Velocity must be in world frame
  static float CalculateTertiaryCost(const Vector2f& robot_position,
                                     const Vector2f& robot_velocity);

  static void AssignDefenders(const state::WorldState& world_state,
                              const state::SoccerState& soccer_state);

 private:
  // Primary Functions only used within Defense Evaluation
  // Calculate the current position of the primary threat. That is the ball or
  // the robot most likely to receive the ball
  static void CalculatePrimaryThreatSource(
      const state::WorldState& world_state);

  static void CalculatePrimaryThreats(const state::WorldState& world_state,
                                      const state::SoccerState& soccer_state);

  // Generate a sorted list of secondary threats
  // Generates at least min_num_threats, though will probably have more
  static void CalculateSecondaryThreats(const state::WorldState& world_state,
                                        const state::SoccerState& soccer_state);

  static float CalculateSecondaryThreatScore(
      const pose_2d::Pose2Df& opponent_pose);


  // Helper Functions only used within Defense Evaluation
  static float CalculatePassRecieveRisk(const Vector2f& ball_position,
                                        const Vector2f& ball_velocity,
                                        const float& ball_speed,
                                        const Vector2f& robot_position);

  static void UpdateRobotIndices(const state::SoccerState& soccer_state);

  // Update primary defender ball interception and assign a robot to intercept
  static void UpdatePrimaryIntercept(const state::WorldState& world_state,
                                     const state::SoccerState& soccer_state);

  // Assign robots to box in the ball if it is close enough to the defense area
  // for the primary defense to not be able to properly retrieve it
  static void BoxBall(const state::WorldState& world_state,
                      const state::SoccerState& soccer_state);

  // Assign robots to block off the open angles to the goal
  static void BlockPrimaryThreats(const state::WorldState& world_state,
                                  const state::SoccerState& soccer_state);

  static void AssignPrimaryDefenders(const state::WorldState& world_state,
                                     const state::SoccerState& soccer_state);

  // Assign primary defenders to block secondary threats
  static void AssignToSecondaryThreats(
      const state::WorldState& world_state,
      const state::SoccerState& soccer_state);

  static void AssignSecondaryDefenders(const state::WorldState& world_state,
                                       const state::SoccerState& soccer_state);

  static void AssignTertiaryDefenders(const state::WorldState& world_state,
                                      const state::SoccerState& soccer_state);

  static bool GetBestTertiaryDefender(const state::WorldState& world_state,
                                      const SecondaryThreat& threat,
                                      const std::vector<DefenderTargetHelper>&
                                          defenders,
                                      unsigned int* defender_index);

  static bool GetBestSecondaryDefender(const state::WorldState& world_state,
                                       const SecondaryThreat& threat,
                                       const std::vector<DefenderTargetHelper>&
                                           defenders,
                                       unsigned int* defender_index);

  // Update the widths of the open angles in the secondary threats
  static void UpdateOpenAngleWidths(const state::WorldState& world_state);

  static void UpdatePreviouslyBlockedThreats(
      const state::WorldState& world_state,
      const state::SoccerState& soccer_state);

  static bool WasBlockingSecondary(const SSLVisionId id,
                                   unsigned int* threat_index,
                                   bool* was_direct);

  // Returns true and populates the threat index if the secondary threat list
  // contains a robot with the specified ID
  static bool GetSecondaryThreat(const SSLVisionId id,
                                 unsigned int* threat_index);

  // Returns the width of the widest open angle on the goal from the input
  // position
  static float GetWidestOpenAngle(
      const Vector2f& position, const std::vector<Vector2f>& blocker_positions);


  // Calculate the distance from the goal position to position a defender given
  // the threat position, goal position and the ratio between the distance from
  // the goal to the defender and the defender to the threat
  static Vector2f CalculateDefenderTarget(
      const Vector2f threat_position,
      const Vector2f goal_position,
      float ratio,
      float max_distance,
      float margin);

  static void UpdatePrimaryDefenseInPlace(const state::WorldState& world_state);

  static bool CanBlockFullAngleSingle(const state::WorldState& world_state,
                                      Vector2f* defender_position);

  // Determine whether a single primary defender and the goalie can block the
  // whole open angle
  static bool CanBlockFullAngleDouble(const state::WorldState& world_state,
                                      Vector2f* defender_position);

  // Determine whether two primary defenders and the goalie can block the
  // whole open angle
  static bool CanBlockFullAngleTriple(const state::WorldState& world_state,
                                      Vector2f* left_position,
                                      Vector2f* right_position);

  static bool CouldIntercept(const Eigen::Vector2f& ball_position,
                             const Eigen::Vector2f& ball_velocity,
                             const Eigen::Vector2f& robot_position,
                             float* relative_percent);

  static void UpdateSecondaryIntercept(const state::WorldState& world_state,
                                       const state::SoccerState& soccer_state);

  static void UpdateTertiaryIntercept(const state::WorldState& world_state,
                                      const state::SoccerState& soccer_state);

  static void MovePrimaryThreat(const Vector2f& goal_intercept,
                                const Vector2f& defender_target);

  // Returns the approximate norm from the rectangle in the direction of the
  // ball
  static Eigen::Vector2f ApproxDefenseAreaNorm(const Vector2f& point);

  static unsigned int num_calculates;
  static unsigned int num_assigns;
  static double calculate_sum;
  static double assign_sum;
  static double calculate_max;
  static double assign_max;


  static logger::Logger defense_logger_;
  static Vector2f primary_threat_position_;
  static Vector2f goalie_position_;
  static bool found_goalie_;

  static unsigned int num_primary_defenders_;
  static unsigned int num_secondary_defenders_;
  static unsigned int num_tertiary_defenders_;

  static std::vector<PrimaryThreat> primary_threats_;
  static std::vector<PrimaryThreat> previous_primary_threats_;

  static std::vector<SecondaryThreat> secondary_threats_;

  static std::vector<SecondaryThreat> previous_blocked_secondary_;

  static bool primary_defender_in_position_;
  static bool primary_threat_is_robot_;
  static SSLVisionId primary_threat_id_;

  static OurRobotIndex goalie_index_;
  static std::vector<OurRobotIndex> primary_defender_indices_;
  static std::vector<OurRobotIndex> secondary_defender_indices_;
  static std::vector<OurRobotIndex> tertiary_defender_indices_;

  static bool primary_intercepting_;
  static int primary_interception_count_;
  static SSLVisionId primary_intercept_id_;
  static pose_2d::Pose2Df previous_intercept_pose_;

  static bool secondary_intercepting_;
  static int secondary_interception_count_;
  static SSLVisionId secondary_intercept_id_;

  static bool tertiary_intercepting_;
  static int tertiary_interception_count_;
  static SSLVisionId tertiary_intercept_id_;


  // Parameters for tuning

  // If the ball is close to the edge of the defense area and there is a single
  // Primary defender, how close to the ball should it get?
  static const float kBlockDistSolitary_;

  // What are the minimum and maximum angles that we expect the opponents to be
  // able to deflect at
  static const float kMinDeflectionAngle_;
  static const float kMaxDeflectionAngle_;
  static const float kDeflectionAngleDiff_;

  // How long do we expect the opponents to take to catch
  static const float kCatchTime_;

  // Assumed kick and pass speeds for opponents for defense calculations (mm/s)
  static const float kOpponentShootSpeed_;
  static const float kOpponentPassSpeed_;

  // Assumed minimum pass distance for opponents (mm)
  static const float kOpponentMinPassDistance_;

  // How fast does the ball have to be moving in order for us to consider
  // forward predicting the primary threat
  static const float kMinPassSpeed_;

  static const float kRiskParam_;
  static const float kMinRisk_;

  // How small of an angle should we consider when assigning secondary defenders
  static const float kMinOpenAngle_;

  // Ratios between the distance of the ball to the defender and the defender to
  // the goal. These may not be constant in the future.
  static const float kPrimaryDefenderRatio_;
  static const float kSecondaryDefenderRatio_;

  // Closest Distance from the defense area that the defenders can be
  static float kPrimaryDefenderMargin_;
  static float kSecondaryDefenderMargin_;

  // What is the maximum distance from the goal that the defenders can be
  // Note that this is ignored if a secondary defender is blocking a pass
  static float kPrimaryDefenderMaxDistance_;
  static float kSecondaryDefenderMaxDistance_;

  // This is mutliplied by the time to other potential poses to determine
  // if a defender should switch poses
  static float kSecondaryCostFactor_;
  static float kTertiaryCostFactor_;

  // How many extra time steps should we keep intercepting
  static const int kInterceptHysterisis_;

  // Minimum squared speed for primary defender interception
  static const float kMinInterceptSpeed_;

  static const float kPrimaryDefenderInflation_;

  static const float kDefenderInPlace_;

  // Relative percentage fro interception
  static const float kMinRelativePercent_;

  // Minimum distance between robots for primary defense
  static const float kMinDistance_;

  // Maximum distance between the defender and the ball for intercepting
  static const float kMaxInterceptDistance_;
};
}  // namespace defense

#endif  // SRC_EVALUATORS_DEFENSE_EVALUATION_H_
