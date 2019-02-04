// Copyright 2017-2018 joydeepb@cs.umass.edu
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

#ifndef SRC_EVALUATORS_OFFENSE_EVALUATION_H_
#define SRC_EVALUATORS_OFFENSE_EVALUATION_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <utility>
#include <vector>
#include "logging/logger.h"
#include "obstacles/obstacle_flag.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "zone/zone.h"

namespace offense {

struct AimOption {
  // The source location.
  Eigen::Vector2f source;
  // The left end of the target segment.
  Eigen::Vector2f target_l;
  // The right end of the target segment.
  Eigen::Vector2f target_r;
  // The center of the target segment.
  Eigen::Vector2f target_center;
  // The intersection between the target line segment and the angle center
  // vector
  Eigen::Vector2f target_bisector;

  // The angle in the direction of middle of the AimOption. Note that this may
  // not be the same as the angle in the direction of target_center since the
  // aim angle does not scale linearly with the target length.
  float angle_center;
  // The angular width of this aim option.
  float angle_width;
  // The value of this aim option.
  float value;

  AimOption()
      : source(0, 0),
        target_l(0, 0),
        target_r(0, 0),
        target_center(0, 0),
        angle_center(0),
        angle_width(0),
        value(0) {}

  AimOption(const Eigen::Vector2f& source, const Eigen::Vector2f& target_left,
            const Eigen::Vector2f& target_right, const float value);
};

void PrintThresholds();

void CalculateAimOptions(const Eigen::Vector2f& source,
                         Eigen::Vector2f target_left,
                         Eigen::Vector2f target_right, float shooting_radius,
                         float obstacle_radius,
                         const std::vector<Eigen::Vector2f>& obstacles,
                         std::vector<AimOption>* aim_options);
// Returns the anglular clearance of the receiver from the passer's viewpoint
float PassEvaluator(const state::WorldState& world_state,
                    SSLVisionId receiver_robot_id, SSLVisionId passing_robot_id,
                    Eigen::Vector2f pass_receiving_pos,
                    float* open_angle_center,
                    const zone::FieldZone& field_zone);

// The same as PassEvaluator with the difference that it does not limit the
// pass receiving area to a specific distace from the receiver
float PassEvaluatorUnlimited(const state::WorldState& world_state,
                             SSLVisionId receiver_robot_id,
                             SSLVisionId passing_robot_id,
                             Eigen::Vector2f pass_receiving_pos,
                             float* open_angle_center);

float GetBestPassTarget(const state::WorldState& world_state,
                        state::SoccerState* soccer_state,
                        const OurRobotIndex& our_robot_index,
                        const SSLVisionId& passing_robot_id,
                        const zone::FieldZone& field_zone,
                        OurRobotIndex* receiving_robot);

float GetBestPassTargetKickoff(const state::WorldState& world_state,
                               state::SoccerState* soccer_state,
                               const OurRobotIndex& our_robot_index,
                               const SSLVisionId& passing_robot_id,
                               const zone::FieldZone& field_zone);

bool SetIsBallMotionObstacle(const state::WorldState& world_state,
                             const OurRobotIndex& our_robot_index,
                             obstacle::ObstacleFlag* obstacle_flag);

void GetTarget(const Vector2f& source, const state::WorldState& world_state,
               state::SoccerState* soccer_state,
               const OurRobotIndex& aiming_robot, const bool& pass_only,
               float* target_angle, OurRobotIndex* target_robot);

float GetTargetEvaluated(const Eigen::Vector2f& source,
                         const state::WorldState& world_state,
                         state::SoccerState* soccer_state,
                         const OurRobotIndex& aiming_robot,
                         const bool& pass_only,
                         Eigen::Vector2f* target_position, float* target_angle,
                         OurRobotIndex* target_robot,
                         OurRobotIndex* last_target);

float GetTargetEvaluated(const Eigen::Vector2f& source,
                         const state::WorldState& world_state,
                         state::SoccerState* soccer_state,
                         const OurRobotIndex& aiming_robot,
                         const bool& pass_only, bool* chip,
                         float* chip_distance, Eigen::Vector2f* target_position,
                         float* target_angle, OurRobotIndex* target_robot,
                         OurRobotIndex* last_target);

bool GetOpenAngleGoal(const Eigen::Vector2f& source,
                      const state::WorldState& world_state,
                      state::SoccerState* soccer_state,
                      const OurRobotIndex& aiming_robot, float* target_angle);

float GetBallTravelTime(const float& ball_speed,
                        const Eigen::Vector2f& ball_pose,
                        const Eigen::Vector2f& final_pose);

float GetFinalBallSpeed(const float& ball_speed, const Vector2f& ball_pose,
                        const Vector2f& final_pose);

float GetNtocTime(const pose_2d::Pose2Df& start,
                  const Eigen::Vector2f& velocity, const Eigen::Vector2f& goal);

float GetBestChipTarget(const state::WorldState& world_state,
                        state::SoccerState* soccer_state,
                        const OurRobotIndex& our_robot_index);

void WritePassFunctionData(
    const state::WorldState& world_state, state::SoccerState* soccer_state,
    float (*score_func)(Vector2f ball_pose,
                        std::vector<unsigned int> robots_to_ignore,
                        Eigen::Vector2f pos_to_eval,
                        const state::WorldState& world_state));

void BuildPassingTable(
    const state::WorldState& world_state, state::SoccerState* soccer_state,
    float (*score_func)(Vector2f ball_pose,
                        std::vector<unsigned int> robots_to_ignore,
                        Eigen::Vector2f pos_to_eval,
                        const state::WorldState& world_state));

float GetPoseCost(Eigen::Vector2f ball,
                  std::vector<unsigned int> robots_to_ignore,
                  Eigen::Vector2f position,
                  const state::WorldState& world_state);

float GetPoseCost(Eigen::Vector2f ball,
                  std::vector<unsigned int> robots_to_ignore,
                  Eigen::Vector2f position,
                  const state::WorldState& world_state, float* aggressive_cost);

// Performs Binary search between the angle of the negative ball velocity
// and the original target vector angle to calculate a new kick angle
// which achieves the target vector given the result of the reflection after
// the kick.
float GetKickAngle(const Eigen::Vector2f& ball_velocity,
                   const Eigen::Vector2f& robot_pose, const float& kick_speed,
                   logger::Logger* robot_logger,
                   const Eigen::Vector2f& target_vector);

static const float kGoalAngleThreshold = DegToRad(1.5);
// Minimum x position on field from which a shot on the goal
// is acceptable.
static const float kGoalDistanceThreshold = -2000;
static const float kMinPassDistance = 1500;
// Used to Calculate the size of the pass array
static constexpr float kSearchOffsetLeft = 2000;
static constexpr float kSearchOffsetRight = 3000;
static constexpr float kSearchOffsetY = 3000;
// Discretization of the pass region.
static constexpr float kStepSize = 90 * 4;
static constexpr int kXSize =
    (kSearchOffsetLeft + kSearchOffsetRight) / kStepSize;
static constexpr int kYSize = 9000 / kStepSize;
// 2 is added to cope with rounding errors;
static constexpr int kPassArraySize = (2 + kXSize) * kYSize;
const float kReceivableThresh = -99999;
static const float kTimingAdjustDistance = 2000;
static const float kTimingAdjustment = 1.2;
static const float kPassSpeed = 2500;
static const float kKickSpeed = 6000;
static const float kDeflectionAngle = DegToRad(70.0);
static const uint kNoPassRobot = 42;
static const float kChippableDistance = 6 * 90;
static const float kPrimaryDefenseDistance = 6 * 90;
static const float kPassChangeThreshold = .05;

// Kick parameters used by all the special/stopped ball
// kicks
const float kThreshAngle = 4;
const float kThreshDistance = 110.0;
const float kThreshYVel = 50;
const float kThreshRelVel = 50;
const float kThreshAlign = 10;
const float kAngularVel = 15.0;

struct PassScore {
  // Pass Score
  float score_;

  // Position
  Vector2f position_;

  PassScore() { score_ = 1; }

  PassScore(const PassScore& pass_score)
      : score_(pass_score.score_), position_(pass_score.position_) {
    score_ = pass_score.score_;
    position_ = pass_score.position_;
  }

  PassScore(const float& score, const Vector2f& position)
      : score_(score), position_(position) {
    score_ = score;
    position_ = position;
  }
};

class TargetEvaluator {
 public:
  TargetEvaluator();
  void Update(const state::WorldState& world_state,
              state::SoccerState* soccer_state,
              float (*score_func)(Vector2f ball_pose,
                                  std::vector<unsigned int> robots_to_ignore,
                                  Eigen::Vector2f pos_to_eval,
                                  const state::WorldState& world_state,
                                  float* aggressive_score));
  Vector2f GetSetupPosition(const OurRobotIndex& robot);

 private:
  const float kSupportAttackerPadding = 2000;
  const float kBallPadding = kMinPassDistance;
  const float kScoreHysterisis = .05;
  std::array<PassScore, kPassArraySize> pass_array_;
  std::array<std::pair<Vector2f, bool>, 8> setup_positions_;
  std::array<float, 8> old_pose_scores_;
  // Used for calculating coercive attacker positions.
  std::array<PassScore, kPassArraySize> aggressive_pass_array_;
  std::array<Vector2f, 8> aggressive_setup_positions_;
  std::array<float, 8> aggressive_old_pose_scores_;
  bool TooClose(const Eigen::Vector2f& pose_1, const Eigen::Vector2f& pose_2);
  bool Blocking(const Vector2f& pose_1, const Vector2f& pose_2,
                const Vector2f& ball_position);
  void ResetPassArray();
  void BuildPassingArray(
      const state::WorldState& world_state, state::SoccerState* soccer_state,
      float (*score_func)(Vector2f ball_pose,
                          std::vector<unsigned int> robots_to_ignore,
                          Eigen::Vector2f pos_to_eval,
                          const state::WorldState& world_state,
                          float* aggressive_cost));
  void AssignSupportPositions(const state::WorldState& world_state,
                              state::SoccerState* soccer_state);
};

}  // namespace offense

#endif  // SRC_EVALUATORS_OFFENSE_EVALUATION_H_
