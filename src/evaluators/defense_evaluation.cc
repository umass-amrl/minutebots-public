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

#include "evaluators/defense_evaluation.h"

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "logging/logger.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "obstacles/obstacle_flag.h"
#include "state/soccer_state.h"
#include "state/world_ball.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/goalie.h"
#include "tactics/primary_defender.h"
#include "tactics/secondary_defender.h"
#include "tactics/tactic_index.h"
#include "tactics/tertiary_defender.h"

STANDARD_USINGS;
using Eigen::Vector2f;
using field_dimensions::kDefenseStretch;
using geometry::Angle;
using geometry::ProjectPointOntoLineSegment;
using geometry::RayIntersect;
using geometry::FurthestFreePointCircle;
using math_util::Sign;
using math_util::Sq;
using geometry::EuclideanDistance;
using geometry::GetNormalizedOrZero;
using geometry::GetTangentPoints;
using geometry::SquaredDistance;
using geometry::SafeVectorNorm;
using logger::Logger;
using motion::MotionModel;
using ntoc::ControlSequence2D;
using ntoc::NTOC2D;
using ntoc::TransformCoordinatesForNTOC;
using obstacle::ObstacleFlag;
using offense::AimOption;
using offense::CalculateAimOptions;
using pose_2d::Pose2Df;
using state::WorldState;
using state::WorldRobot;
using state::SoccerRobot;
using state::SoccerState;
using std::array;
using std::cos;
using std::find;
using std::numeric_limits;
using std::sin;
using std::pair;
using tactics::Goalie;
using tactics::PrimaryDefender;
using tactics::SecondaryDefender;
using tactics::TertiaryDefender;
using tactics::TacticIndex;

namespace defense {
unsigned int DefenseEvaluator::num_calculates = 0;
unsigned int DefenseEvaluator::num_assigns = 0;
double DefenseEvaluator::calculate_sum = 0;
double DefenseEvaluator::assign_sum = 0;
double DefenseEvaluator::calculate_max = 0;
double DefenseEvaluator::assign_max = 0;

// Parameters for tuning
// If the ball is close to the edge of the defense area and there is a single
// Primary defender, how close to the ball should it get?
const float DefenseEvaluator::kBlockDistSolitary_ =
    kRobotRadius + kBallRadius + 20.0f;

// What are the minimum and maximum angles that we expect the opponents to be
// able to deflect at
const float DefenseEvaluator::kMinDeflectionAngle_ = DegToRad(45);
const float DefenseEvaluator::kMaxDeflectionAngle_ = DegToRad(90);
const float DefenseEvaluator::kDeflectionAngleDiff_ =
    AngleDiff(DefenseEvaluator::kMaxDeflectionAngle_,
              DefenseEvaluator::kMinDeflectionAngle_);

// How long do we expect the opponents to take to catch
const float DefenseEvaluator::kCatchTime_ = 0.5;

// Assumed kick and pass speeds for opponents for defense calculations (mm/s)
const float DefenseEvaluator::kOpponentShootSpeed_ = 6500.0f;
const float DefenseEvaluator::kOpponentPassSpeed_ = 4000.0f;

// Assumed minimum pass distance for opponents (mm)
const float DefenseEvaluator::kOpponentMinPassDistance_ = 1000.0f;

// How fast does the ball have to be moving in order for us to consider
// forward predicting the primary threat
const float DefenseEvaluator::kMinPassSpeed_ = 500.0f;

const float DefenseEvaluator::kRiskParam_ = 10.0;
const float DefenseEvaluator::kMinRisk_ = 5.0;

// How small of an angle should we consider when assigning secondary defenders
const float DefenseEvaluator::kMinOpenAngle_ = DegToRad(1.0);

// Ratios between the distance of the ball to the defender and the defender to
// the goal. These may not be constant in the future.
const float DefenseEvaluator::kPrimaryDefenderRatio_ = 0.45;
const float DefenseEvaluator::kSecondaryDefenderRatio_ = 1.2;

// Closest Distance from the defense area that the defenders can be
float DefenseEvaluator::kPrimaryDefenderMargin_ = kRobotRadius + 25 + kEpsilon;
float DefenseEvaluator::kSecondaryDefenderMargin_ =
    2.0 * kRobotRadius + 50 + kEpsilon;

// What is the maximum distance from the goal that the defenders can be
// Note that this is ignored if a secondary defender is blocking a pass
float DefenseEvaluator::kPrimaryDefenderMaxDistance_ = 2000.0f;
float DefenseEvaluator::kSecondaryDefenderMaxDistance_ = 3000.0f;

// This is mutliplied by the time to other potential poses to determine
// if a defender should switch poses
float DefenseEvaluator::kSecondaryCostFactor_ = 0.5;
float DefenseEvaluator::kTertiaryCostFactor_ = 0.5;

// How many extra time steps should we keep intercepting
const int DefenseEvaluator::kInterceptHysterisis_ = 10;

const float DefenseEvaluator::kMinInterceptSpeed_ = 250;  // 250 mm/s squared;

const float DefenseEvaluator::kPrimaryDefenderInflation_ =
    kRobotRadius + 25 + kEpsilon;

const float DefenseEvaluator::kDefenderInPlace_ = 100.0;

// Relative percentage fro interception
const float DefenseEvaluator::kMinRelativePercent_ = 80.0;

// Minimum distance between robots for primary defense
const float DefenseEvaluator::kMinDistance_ = 40.0;

const float DefenseEvaluator::kMaxInterceptDistance_ = 3000.0;

PrimaryThreat::PrimaryThreat()
    : defender_pose_(0, 0, 0),
      is_blocked_(false),
      blocking_robot_id_(42),
      is_intercept_(false),
      notify_goalie_(false) {}

SecondaryThreat::SecondaryThreat()
    : direct_blocked_(false),
      indirect_blocked_(false),
      score_(numeric_limits<float>::max()),
      threat_position_(0, 0),
      primary_target_(0, 0),
      secondary_target_(0, 0),
      tertiary_target_(0, 0),
      open_angle_width_(0),
      robot_id_(42),
      direct_blocking_id_(42),
      indirect_blocking_id_(42) {}

DefenderTargetHelper::DefenderTargetHelper()
    : is_assigned_(false),
      could_be_improved_(true),
      is_direct_(false),
      score_(0),
      threat_index_(0),
      is_intercepting_(false) {}

DefenseEvaluator::DefenseEvaluator() {}

bool DefenseEvaluator::PointInDefenseArea(const Vector2f& point, float margin,
                                          bool our_defense_area) {
  ObstacleFlag defense_area;
  if (our_defense_area) {
    defense_area = ObstacleFlag::GetOurDefenseArea();
  } else {
    defense_area = ObstacleFlag::GetTheirDefenseArea();
  }
  bool is_inside = false;
  for (const auto& obstacle : defense_area) {
    if (obstacle->PointCollision(point, margin)) {
      is_inside = true;
    }
  }

  return is_inside;
}

void DefenseEvaluator::SetGoalie(const WorldState& world_state) {
  if (Goalie::goalie_set_) {
    for (const auto& robot : world_state.GetOurRobots()) {
      if (robot.confidence > 0 &&
          robot.ssl_vision_id == Goalie::goalie_ssl_id_) {
        goalie_position_ = robot.position.translation;
        return;
        break;
      }
    }
  }

  float closest_squared_distance = 10e20;
  // If you have gotten here, the goalie has either not been assigned or the id
  // is no longer present, pick a new goalie
  for (OurRobotIndex index = 0; index < world_state.GetNumOurRobots();
       index++) {
    const Pose2Df& current_pose =
        world_state.GetOurRobotPosition(index).position;

    float current_squared_distance =
        SquaredDistance(current_pose.translation, kOurGoalCenter);

    if (current_squared_distance < closest_squared_distance) {
      closest_squared_distance = current_squared_distance;
      Goalie::goalie_ssl_id_ =
          world_state.GetOurRobotPosition(index).ssl_vision_id;

      goalie_position_ =
          world_state.GetOurRobotPosition(index).position.translation;
    }
  }
}

bool DefenseEvaluator::BallMayEnterGoal(const Vector2f ball_position,
                                        const Vector2f ball_velocity,
                                        bool is_ours) {
  if (is_ours) {
    return RayIntersect(ball_position, ball_velocity, kOurGoalL, kOurGoalR);
  } else {
    return RayIntersect(ball_position, ball_velocity, kTheirGoalL, kTheirGoalR);
  }
}

void DefenseEvaluator::ResetDefenseLogger() {
  defense_logger_.Clear();
  defense_logger_.LogPrintPush("Defense Evaluation");
}

const Logger& DefenseEvaluator::GetDefenseLogger() {
  defense_logger_.Pop();
  return defense_logger_;
}

void DefenseEvaluator::UpdateNumDefenders(const vector<string>& role_list) {
  num_primary_defenders_ = 0;
  num_secondary_defenders_ = 0;
  num_tertiary_defenders_ = 0;

  for (const string& role : role_list) {
    if (!role.compare("primary_defender")) {
      num_primary_defenders_++;
    } else if (!role.compare("secondary_defender")) {
      num_secondary_defenders_++;
    } else if (!role.compare("tertiary_defender")) {
      num_tertiary_defenders_++;
    }
  }
}

void DefenseEvaluator::CalculateThreats(const WorldState& world_state,
                                        const SoccerState& soccer_state) {
  double t_start = GetMonotonicTime();
  ResetDefenseLogger();
  primary_threats_.clear();
  secondary_threats_.clear();
  CalculatePrimaryThreatSource(world_state);
  CalculatePrimaryThreats(world_state, soccer_state);
  CalculateSecondaryThreats(world_state, soccer_state);
  double t_end = GetMonotonicTime();
  num_calculates++;
  double time = t_end - t_start;
  if (time > calculate_max) calculate_max = time;
  calculate_sum += time;
  defense_logger_.LogPrintPush("Calculate Threat Timing");
  defense_logger_.LogPrint("Current Runtime: %f", time);
  defense_logger_.LogPrint("Average Runtime: %f",
                           calculate_sum / static_cast<float>(num_calculates));
  defense_logger_.LogPrint("Max Runtime: %f", calculate_max);
  defense_logger_.Pop();
}

float DefenseEvaluator::CalculatePrimaryCost(const Vector2f& robot_position,
                                             const Vector2f& robot_velocity) {
  ControlSequence2D sequence;
  MotionModel model(kDefaultRobotVelocity, kDefaultRobotAcceleration);

  float min_time = 10e20;
  unsigned int num_examined = 0;

  // Go through direct targets
  for (const PrimaryThreat& threat : primary_threats_) {
    Vector2f target = threat.defender_pose_.translation;

    float time =
        NTOC2D(robot_position - target, robot_velocity, model, &sequence);

    if (time < min_time) {
      min_time = time;
    }

    num_examined++;
    if (num_examined >= num_primary_defenders_) {
      return min_time;
    }
  }

  if (num_examined > 0) return min_time;

  // If all else fails, return time to goal
  return NTOC2D(robot_position - kOurGoalCenter, robot_velocity, model,
                &sequence);
}

float DefenseEvaluator::CalculateSecondaryCost(SSLVisionId id,
                                               const Vector2f& robot_position,
                                               const Vector2f& robot_velocity) {
  static const bool kDebug = false;
  float min_time = 10e20;
  unsigned int num_examined = 0;

  ControlSequence2D sequence;
  MotionModel model(kDefaultRobotVelocity, kDefaultRobotAcceleration);

  if (kDebug) {
    defense_logger_.LogPrintPush("Secondary Cost: Robot %d", id);
  }

  // Go through direct targets
  for (const SecondaryThreat& threat : secondary_threats_) {
    if (threat.open_angle_width_ < kMinOpenAngle_) continue;

    Vector2f target = threat.secondary_target_;

    float time =
        NTOC2D(robot_position - target, robot_velocity, model, &sequence);

    if (time < min_time) {
      min_time = time;
    }

    if (kDebug) {
      defense_logger_.LogPrint("Target: %f, %f", target.x(), target.y());
      defense_logger_.LogPrint("Time: %f", time);
    }

    num_examined++;
    if (num_examined >= num_secondary_defenders_) {
      if (kDebug) {
        defense_logger_.Pop();
      }
      return min_time;
    }
  }

  // Go through indirect targets if necessary
  for (const SecondaryThreat& threat : secondary_threats_) {
    if (threat.open_angle_width_ < kMinOpenAngle_) continue;
    Vector2f target = threat.tertiary_target_;

    float time =
        NTOC2D(robot_position - target, robot_velocity, model, &sequence);

    if (time < min_time) {
      min_time = time;
    }

    if (kDebug) {
      defense_logger_.LogPrint("Target: %f, %f", target.x(), target.y());
      defense_logger_.LogPrint("Time: %f", time);
    }

    num_examined++;
    if (num_examined >= num_secondary_defenders_) {
      if (kDebug) {
        defense_logger_.Pop();
      }
      return min_time;
    }
  }

  // Same thing but now look at robots without an open angle
  for (const SecondaryThreat& threat : secondary_threats_) {
    Vector2f target = threat.tertiary_target_;

    float time =
        NTOC2D(robot_position - target, robot_velocity, model, &sequence);

    if (time < min_time) {
      min_time = time;
    }

    if (kDebug) {
      defense_logger_.LogPrint("Target: %f, %f", target.x(), target.y());
      defense_logger_.LogPrint("Time: %f", time);
    }

    num_examined++;
    if (num_examined >= num_secondary_defenders_) {
      if (kDebug) {
        defense_logger_.Pop();
      }
      return min_time;
    }
  }

  for (const SecondaryThreat& threat : secondary_threats_) {
    Vector2f target = threat.tertiary_target_;

    float time =
        NTOC2D(robot_position - target, robot_velocity, model, &sequence);

    if (time < min_time) {
      min_time = time;
    }

    if (kDebug) {
      defense_logger_.LogPrint("target: %f, %f", target.x(), target.y());
      defense_logger_.LogPrint("Time: %f", time);
    }
    num_examined++;
    if (num_examined >= num_secondary_defenders_) {
      if (kDebug) {
        defense_logger_.Pop();
      }
      return min_time;
    }
  }

  if (num_examined > 0) {
    defense_logger_.Pop();
    return min_time;
  }

  if (kDebug) {
    defense_logger_.LogPrint("Returning time to goal");
    defense_logger_.Pop();
  }
  // If all else fails, return time to goal
  return NTOC2D(robot_position - kOurGoalCenter, robot_velocity, model,
                &sequence);
}

float DefenseEvaluator::CalculateTertiaryCost(const Vector2f& robot_position,
                                              const Vector2f& robot_velocity) {
  float min_time = 10e20;
  unsigned int num_examined = 0;

  ControlSequence2D sequence;
  MotionModel model(kDefaultRobotVelocity, kDefaultRobotAcceleration);

  // Go through indirect targets
  for (const SecondaryThreat& threat : secondary_threats_) {
    Vector2f target = threat.tertiary_target_;

    float time =
        NTOC2D(robot_position - target, robot_velocity, model, &sequence);

    if (time < min_time) {
      min_time = time;
    }

    num_examined++;
    if (num_examined >= num_secondary_defenders_) {
      return min_time;
    }
  }

  if (num_examined > 0) return min_time;

  // If all else fails, return time to goal
  return NTOC2D(robot_position - kOurGoalCenter, robot_velocity, model,
                &sequence);
}

void DefenseEvaluator::CalculatePrimaryThreatSource(
    const WorldState& world_state) {
  static const bool kDebug = true;
  if (kDebug) {
    defense_logger_.LogPrintPush("Calculate Primary Threat");
  }
  const Vector2f& ball_position = world_state.GetBallPosition().position;
//   const Vector2f& ball_velocity = world_state.GetBallPosition().velocity;
//   const float ball_speed = SafeVectorNorm(ball_velocity);
//
//   Vector2f threat_position(ball_position);
//   primary_threat_is_robot_ = false;
//
//   if (kDebug) {
//     defense_logger_.LogPrint("Min Risk Threshold: %f", kMinRisk_);
//   }
//
//   if (!BallMayEnterGoal(ball_position, ball_velocity, true) &&
//       ball_speed >= kMinPassSpeed_) {
//     float max_risk = kMinRisk_;
//     Vector2f max_threat = ball_position;
//     for (const auto& robot : world_state.GetTheirRobots()) {
//       Vector2f robot_position = robot.position.translation;
//       float risk = CalculatePassRecieveRisk(ball_position, ball_velocity,
//                                             ball_speed, robot_position);
//       if (kDebug) {
//         defense_logger_.LogPrint("Robot %d: %f", robot.ssl_vision_id, risk);
//       }
//       if (risk >= max_risk) {
//         max_risk = risk;
//         max_threat = robot_position;
//         primary_threat_is_robot_ = true;
//         primary_threat_id_ = robot.ssl_vision_id;
//       }
//     }
//
//     defense_logger_.AddCircle(threat_position, 20, 0, 0, 1, 0.75);
//     defense_logger_.LogPrint("Primary Threat Location: %f, %f",
//                              threat_position.x(), threat_position.y());
//     if (primary_threat_is_robot_) {
//       defense_logger_.LogPrint("Primary Threat Robot ID: %d",
//                                primary_threat_id_);
//     }
//
//     threat_position = max_threat;
//   }
//
//   if (kDebug) {
//     defense_logger_.Pop();
//   }

  primary_threat_position_ = ball_position;
}

void DefenseEvaluator::CalculatePrimaryThreats(
    const WorldState& world_state, const SoccerState& soccer_state) {
  if (num_primary_defenders_ > 0) {
    if (soccer_state.IsNormalPlay()) {
      BoxBall(world_state, soccer_state);
    }
    if (primary_threats_.empty()) {
      BlockPrimaryThreats(world_state, soccer_state);
    }
  }
}

void DefenseEvaluator::CalculateSecondaryThreats(
    const WorldState& world_state, const SoccerState& soccer_state) {
  // Empty secondary threat list
  secondary_threats_.clear();
  vector<Vector2f> blocker_positions;
  if (found_goalie_) {
    blocker_positions.push_back(goalie_position_);
  }

  defense_logger_.LogPrintPush("Secondary Threats");

  Vector2f ball_pos = world_state.GetBallPosition().position;
  float right_angle = Angle(Vector2f(kTheirGoalR - ball_pos));
  float left_angle = Angle(Vector2f(kTheirGoalL - ball_pos));

  for (const auto& robot : world_state.GetTheirRobots()) {
    if (robot.confidence > kEpsilon) {
      Pose2Df opponent_pose = robot.position;
      float opponent_angle = Angle(Vector2f(opponent_pose.translation));

      if (EuclideanDistance(opponent_pose.translation,
                            primary_threat_position_) >
              kOpponentMinPassDistance_ &&
          !PointInDefenseArea(opponent_pose.translation, 0.0, false) &&
          (opponent_angle < right_angle || opponent_angle > left_angle)) {
        // Make a secondary threat for the line from the opponent to the goal
        SecondaryThreat threat;
        threat.direct_blocked_ = false;
        threat.indirect_blocked_ = false;
        threat.score_ = CalculateSecondaryThreatScore(opponent_pose);
        threat.threat_position_ = opponent_pose.translation;
        threat.primary_target_ = CalculateDefenderTarget(
            opponent_pose.translation, kOurGoalCenter, kPrimaryDefenderRatio_,
            kPrimaryDefenderMaxDistance_, kPrimaryDefenderMargin_);
        threat.secondary_target_ = CalculateDefenderTarget(
            opponent_pose.translation, kOurGoalCenter, kSecondaryDefenderRatio_,
            kSecondaryDefenderMaxDistance_, kSecondaryDefenderMargin_);

        // Tertiary Target is halfway between the primary threat and the
        // opponent
        threat.tertiary_target_ =
            (opponent_pose.translation + primary_threat_position_) / 2.0;

        threat.open_angle_width_ =
            GetWidestOpenAngle(opponent_pose.translation, blocker_positions);
        threat.robot_id_ = robot.ssl_vision_id;

        threat.direct_blocking_id_ = 42;
        threat.indirect_blocking_id_ = 42;

        secondary_threats_.push_back(threat);

        defense_logger_.LogPrint("Robot %d: %f", robot.ssl_vision_id,
                                 threat.score_);
      }
    }
  }

  defense_logger_.Pop();

  std::sort(secondary_threats_.begin(), secondary_threats_.end());
}

void DefenseEvaluator::UpdateRobotIndices(const SoccerState& soccer_state) {
  primary_defender_indices_.clear();
  secondary_defender_indices_.clear();
  tertiary_defender_indices_.clear();
  goalie_index_ = 42;
  found_goalie_ = false;

  num_primary_defenders_ = 0;
  num_secondary_defenders_ = 0;
  num_tertiary_defenders_ = 0;

  for (const SoccerRobot& robot : soccer_state.GetAllSoccerRobots()) {
    if (robot.enabled_) {
      if (robot.current_tactic_ == tactics::GOALIE) {
        goalie_index_ = robot.our_robot_index_;
        found_goalie_ = true;
      } else if (robot.current_tactic_ == tactics::PRIMARY_DEFENDER) {
        primary_defender_indices_.push_back(robot.our_robot_index_);
        num_primary_defenders_++;
      } else if (robot.current_tactic_ == tactics::SECONDARY_DEFENDER) {
        secondary_defender_indices_.push_back(robot.our_robot_index_);
        num_secondary_defenders_++;
      } else if (robot.current_tactic_ == tactics::TERTIARY_DEFENDER) {
        tertiary_defender_indices_.push_back(robot.our_robot_index_);
        num_tertiary_defenders_++;
      }
    }
  }
}

void DefenseEvaluator::AssignDefenders(const WorldState& world_state,
                                       const SoccerState& soccer_state) {
  double t_start = GetMonotonicTime();
  UpdateRobotIndices(soccer_state);
  defense_logger_.LogPrintPush("Assign Defenders");
  defense_logger_.LogPrint("Num Primary Defenders: %d", num_primary_defenders_);
  defense_logger_.LogPrint("Num Secondary Defenders: %d",
                           num_secondary_defenders_);
  defense_logger_.LogPrint("Num Tertiary Defenders: %d",
                           num_tertiary_defenders_);
  if (found_goalie_) {
    const SoccerRobot& goalie =
        soccer_state.GetRobotByOurRobotIndex(goalie_index_);
    goalie.SetGoal(Pose2Df(0.0f, primary_threat_position_));
  }
  UpdatePrimaryIntercept(world_state, soccer_state);
  if (num_primary_defenders_ > 0) {
    AssignPrimaryDefenders(world_state, soccer_state);
    UpdateOpenAngleWidths(world_state);
    UpdatePrimaryDefenseInPlace(world_state);
  }
  UpdateSecondaryIntercept(world_state, soccer_state);
  if (num_secondary_defenders_ > 0) {
    AssignSecondaryDefenders(world_state, soccer_state);
  }
  UpdateTertiaryIntercept(world_state, soccer_state);
  if (num_tertiary_defenders_ > 0) {
    AssignTertiaryDefenders(world_state, soccer_state);
  }
  UpdatePreviouslyBlockedThreats(world_state, soccer_state);
  double t_end = GetMonotonicTime();
  num_assigns++;
  double time = t_end - t_start;
  if (time > assign_max) assign_max = time;
  assign_sum += time;
  defense_logger_.LogPrintPush("Assign Defender Timing");
  defense_logger_.LogPrint("Current Runtime: %f", time);
  defense_logger_.LogPrint("Average Runtime: %f",
                           assign_sum / static_cast<float>(num_assigns));
  defense_logger_.LogPrint("Max Runtime: %f", assign_max);
  defense_logger_.Pop();
}

void DefenseEvaluator::AssignPrimaryDefenders(const WorldState& world_state,
                                              const SoccerState& soccer_state) {
  // Calculate Primary Defender Angles
  if (primary_defender_indices_.size() > primary_threats_.size()) {
    LOG(ERROR) << "More primary defenders than threats";
  }

  vector<pair<OurRobotIndex, float>> defender_angles;
  for (const OurRobotIndex& index : primary_defender_indices_) {
    pair<OurRobotIndex, float> defender_pair;
    defender_pair.first = index;
    defender_pair.second = Angle(
        Vector2f(world_state.GetOurRobotPosition(index).position.translation -
                 kOurGoalCenter));
    defender_angles.push_back(defender_pair);
  }

  // Calculate Primary Threat Angles
  vector<pair<unsigned int, float>> threat_angles;
  for (unsigned int i = 0; i < primary_threats_.size(); i++) {
    pair<unsigned int, float> threat_pair;
    threat_pair.first = i;
    threat_pair.second = Angle(Vector2f(
        primary_threats_[i].defender_pose_.translation - kOurGoalCenter));
    threat_angles.push_back(threat_pair);
  }

  std::sort(defender_angles.begin(), defender_angles.end(),
            [](pair<OurRobotIndex, float> a, pair<OurRobotIndex, float> b) {
              return a.second < b.second;
            });

  std::sort(threat_angles.begin(), threat_angles.end(),
            [](pair<OurRobotIndex, float> a, pair<OurRobotIndex, float> b) {
              return a.second < b.second;
            });

  // Assign robots to threats
  for (unsigned int i = 0; i < defender_angles.size(); i++) {
    OurRobotIndex index = defender_angles[i].first;
    const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(index);
    SSLVisionId id = world_state.GetOurRobotPosition(index).ssl_vision_id;

    if (id == primary_intercept_id_ && primary_intercepting_) {
      defender.SetGoal(previous_intercept_pose_);
      static_cast<PrimaryDefender*>(
          defender.tactic_list_[tactics::PRIMARY_DEFENDER].get())
          ->SetIntercept();

      return;
    }

    if (i >= threat_angles.size()) {
      defense_logger_.LogPrint("Assigning robot %d to default position",
                               defender.ssl_vision_id_);
      // There are not enough threats, assign to default position
      Vector2f position = CalculateDefenderTarget(
          Vector2f(0, 0), kOurGoalCenter, kPrimaryDefenderRatio_,
          kPrimaryDefenderMaxDistance_, kPrimaryDefenderInflation_);

      Pose2Df default_pose(0, position);

      defender.SetGoal(default_pose);
      return;
    }

    unsigned int threat_index = threat_angles[i].first;
    defender.SetGoal(primary_threats_[threat_index].defender_pose_);
    primary_threats_[threat_index].is_blocked_ = true;
    primary_threats_[threat_index].blocking_robot_id_ =
        world_state.GetOurRobotPosition(index).ssl_vision_id;

    if (primary_threats_[threat_index].notify_goalie_) {
      // Notify goalie it should factor this robot into its open angle
      // calculation
      if (found_goalie_) {
        const SoccerRobot& goalie =
            soccer_state.GetRobotByOurRobotIndex(goalie_index_);
        static_cast<Goalie*>(goalie.tactic_list_[tactics::GOALIE].get())
            ->AddDefender(index);
      }
    }
  }
}

// TODO(slane): Split this into helper functions
void DefenseEvaluator::AssignSecondaryDefenders(
    const WorldState& world_state, const SoccerState& soccer_state) {
  // Start with the robots that were present in the last set of tertiary
  // defenders
  vector<DefenderTargetHelper> defenders;
  for (const OurRobotIndex& index : secondary_defender_indices_) {
    const SSLVisionId& id =
        world_state.GetOurRobotPosition(index).ssl_vision_id;
    unsigned int threat_index;
    DefenderTargetHelper helper;
    helper.ssl_id_ = id;
    if (id == secondary_intercept_id_) {
      helper.is_assigned_ = true;
      helper.score_ = 0.0;
      helper.could_be_improved_ = false;
      helper.is_intercepting_ = true;
      bool is_direct;
      if (WasBlockingSecondary(id, &threat_index, &is_direct)) {
        if (is_direct) {
          secondary_threats_[threat_index].direct_blocked_ = true;
          secondary_threats_[threat_index].direct_blocking_id_ = id;
        } else {
          secondary_threats_[threat_index].indirect_blocked_ = true;
          secondary_threats_[threat_index].indirect_blocking_id_ = id;
        }
      }

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(index);
      static_cast<SecondaryDefender*>(
          defender.tactic_list_[tactics::SECONDARY_DEFENDER].get())
          ->SetIntercept();
    }

    if (WasBlockingSecondary(id, &threat_index, &helper.is_direct_)) {
      helper.is_assigned_ = true;
      helper.score_ = secondary_threats_[threat_index].score_;
      helper.threat_index_ = threat_index;

      if (primary_threat_id_ == secondary_threats_[threat_index].robot_id_ &&
          helper.is_direct_ && !primary_defender_in_position_) {
        helper.could_be_improved_ = false;
      }
    }
    defenders.push_back(helper);
  }

  // Go through the direct threats and assign defenders to those
  for (unsigned int i = 0; i < secondary_threats_.size(); i++) {
    SecondaryThreat* threat = &secondary_threats_[i];
    if (threat->direct_blocked_ || threat->open_angle_width_ < kMinOpenAngle_)
      continue;
    unsigned int desired_defender;
    bool should_block = GetBestSecondaryDefender(world_state, *threat,
                                                 defenders, &desired_defender);

    if (should_block) {
      defenders[desired_defender].is_assigned_ = true;
      defenders[desired_defender].could_be_improved_ = false;
      defenders[desired_defender].score_ = 0;
      defenders[desired_defender].threat_index_ = i;
      threat->direct_blocked_ = true;
      threat->direct_blocking_id_ = defenders[desired_defender].ssl_id_;

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(
          secondary_defender_indices_[desired_defender]);
      Pose2Df defensive_pose(
          Angle(Vector2f(primary_threat_position_ - threat->secondary_target_)),
          threat->secondary_target_);
      defender.SetGoal(defensive_pose);
    }
  }

  // Check that all robots have been assigned
  bool all_assigned = true;
  for (unsigned int i = 0; i < defenders.size(); i++) {
    if (defenders[i].could_be_improved_) {
      all_assigned = false;
      break;
    }
  }

  if (all_assigned) return;

  // Go through indirect threats and assign defenders to those
  for (unsigned int i = 0; i < secondary_threats_.size(); i++) {
    SecondaryThreat* threat = &secondary_threats_[i];
    if (threat->indirect_blocked_ || threat->open_angle_width_ < kMinOpenAngle_)
      continue;
    unsigned int desired_defender;
    bool should_block = GetBestSecondaryDefender(world_state, *threat,
                                                 defenders, &desired_defender);

    if (should_block) {
      defenders[desired_defender].is_assigned_ = true;
      defenders[desired_defender].could_be_improved_ = false;
      defenders[desired_defender].score_ = 0;
      defenders[desired_defender].threat_index_ = i;
      threat->indirect_blocked_ = true;
      threat->indirect_blocking_id_ = defenders[desired_defender].ssl_id_;

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(
          secondary_defender_indices_[desired_defender]);
      Pose2Df defensive_pose(
          Angle(Vector2f(primary_threat_position_ - threat->tertiary_target_)),
          threat->tertiary_target_);
      defender.SetGoal(defensive_pose);
    }
  }

  // Check that all robots have been assigned
  all_assigned = true;
  for (unsigned int i = 0; i < defenders.size(); i++) {
    if (defenders[i].could_be_improved_) {
      all_assigned = false;
      break;
    }
  }

  if (all_assigned) return;

  // Assign to direct threats that are not wide enough
  // Go through the direct threats and assign defenders to those
  for (unsigned int i = 0; i < secondary_threats_.size(); i++) {
    SecondaryThreat* threat = &secondary_threats_[i];
    if (threat->direct_blocked_) continue;
    unsigned int desired_defender;
    bool should_block = GetBestSecondaryDefender(world_state, *threat,
                                                 defenders, &desired_defender);

    if (should_block) {
      defenders[desired_defender].is_assigned_ = true;
      defenders[desired_defender].could_be_improved_ = false;
      defenders[desired_defender].score_ = 0;
      defenders[desired_defender].threat_index_ = i;
      threat->direct_blocked_ = true;
      threat->direct_blocking_id_ = defenders[desired_defender].ssl_id_;

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(
          secondary_defender_indices_[desired_defender]);
      Pose2Df defensive_pose(
          Angle(Vector2f(primary_threat_position_ - threat->secondary_target_)),
          threat->secondary_target_);
      defender.SetGoal(defensive_pose);
    }
  }

  // Check that all robots have been assigned
  all_assigned = true;
  for (unsigned int i = 0; i < defenders.size(); i++) {
    if (defenders[i].could_be_improved_) {
      all_assigned = false;
      break;
    }
  }

  // Assign to indirect threats that are not wide enough
  for (unsigned int i = 0; i < secondary_threats_.size(); i++) {
    SecondaryThreat* threat = &secondary_threats_[i];
    if (threat->indirect_blocked_) continue;
    unsigned int desired_defender;
    bool should_block = GetBestSecondaryDefender(world_state, *threat,
                                                 defenders, &desired_defender);

    if (should_block) {
      defenders[desired_defender].is_assigned_ = true;
      defenders[desired_defender].could_be_improved_ = false;
      defenders[desired_defender].score_ = 0;
      defenders[desired_defender].threat_index_ = i;
      threat->indirect_blocked_ = true;
      threat->indirect_blocking_id_ = defenders[desired_defender].ssl_id_;

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(
          secondary_defender_indices_[desired_defender]);
      Pose2Df defensive_pose(
          Angle(Vector2f(primary_threat_position_ - threat->tertiary_target_)),
          threat->tertiary_target_);
      defender.SetGoal(defensive_pose);
    }
  }

  // TODO(slane): Assign remaining secondary defenders to sensible positions
  for (unsigned int i = 0; i < defenders.size(); i++) {
    if (defenders[i].could_be_improved_) {
      const SoccerRobot& defender =
          soccer_state.GetRobotByOurRobotIndex(secondary_defender_indices_[i]);
      Pose2Df defensive_pose(0, 0, 0);
      defender.SetGoal(defensive_pose);
    }
  }
}

void DefenseEvaluator::AssignTertiaryDefenders(
    const WorldState& world_state, const SoccerState& soccer_state) {
  vector<DefenderTargetHelper> defenders;

  defense_logger_.LogPrintPush("Assign Tertiary Defenders");
  defense_logger_.LogPrint("Num Secondary Threats: %d",
                           secondary_threats_.size());
  defense_logger_.LogPrint("Num Tertiary Defenders: %d",
                           num_tertiary_defenders_);

  // Start with the robots that were present in the last set of tertiary
  // defenders
  for (const OurRobotIndex& index : tertiary_defender_indices_) {
    const SSLVisionId& id =
        world_state.GetOurRobotPosition(index).ssl_vision_id;
    unsigned int threat_index;
    DefenderTargetHelper helper;
    helper.ssl_id_ = id;
    bool is_direct;
    if (id == tertiary_intercept_id_) {
      helper.is_assigned_ = true;
      helper.score_ = 0.0;
      helper.could_be_improved_ = false;
      helper.is_intercepting_ = true;

      if (WasBlockingSecondary(id, &threat_index, &is_direct)) {
        secondary_threats_[threat_index].indirect_blocked_ = true;
        secondary_threats_[threat_index].indirect_blocking_id_ = id;
      }

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(index);
      static_cast<TertiaryDefender*>(
          defender.tactic_list_[tactics::TERTIARY_DEFENDER].get())
          ->SetIntercept();
    }

    if (WasBlockingSecondary(id, &threat_index, &is_direct)) {
      helper.is_assigned_ = true;
      helper.score_ = secondary_threats_[threat_index].score_;
      helper.threat_index_ = threat_index;
    }
    defenders.push_back(helper);
  }

  // Start with threats that have not had their direct line blocked
  for (unsigned int i = 0; i < secondary_threats_.size(); i++) {
    SecondaryThreat* threat = &secondary_threats_[i];
    if (threat->indirect_blocked_ || threat->direct_blocked_) continue;
    unsigned int desired_defender;
    bool should_block = GetBestTertiaryDefender(world_state, *threat, defenders,
                                                &desired_defender);

    if (should_block) {
      defenders[desired_defender].is_assigned_ = true;
      defenders[desired_defender].could_be_improved_ = false;
      defenders[desired_defender].score_ = 0;
      defenders[desired_defender].threat_index_ = i;
      threat->indirect_blocked_ = true;
      threat->indirect_blocking_id_ = defenders[desired_defender].ssl_id_;

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(
          tertiary_defender_indices_[desired_defender]);
      Pose2Df defensive_pose(
          Angle(Vector2f(primary_threat_position_ - threat->tertiary_target_)),
          threat->tertiary_target_);
      defender.SetGoal(defensive_pose);
    }
  }

  // Now block robots that have had their direct blocked
  for (unsigned int i = 0; i < secondary_threats_.size(); i++) {
    SecondaryThreat* threat = &secondary_threats_[i];
    if (threat->indirect_blocked_) continue;
    unsigned int desired_defender;
    bool should_block = GetBestTertiaryDefender(world_state, *threat, defenders,
                                                &desired_defender);

    if (should_block) {
      defenders[desired_defender].is_assigned_ = true;
      defenders[desired_defender].could_be_improved_ = false;
      defenders[desired_defender].score_ = 0;
      defenders[desired_defender].threat_index_ = i;
      threat->indirect_blocked_ = true;
      threat->indirect_blocking_id_ = defenders[desired_defender].ssl_id_;

      const SoccerRobot& defender = soccer_state.GetRobotByOurRobotIndex(
          tertiary_defender_indices_[desired_defender]);
      Pose2Df defensive_pose(
          Angle(Vector2f(primary_threat_position_ - threat->tertiary_target_)),
          threat->tertiary_target_);
      defender.SetGoal(defensive_pose);
    }
  }

  // TODO(slane): Assign defenders that have not yet been assigned to something
  // to some sensible position
  for (unsigned int i = 0; i < defenders.size(); i++) {
    if (defenders[i].could_be_improved_) {
      const SoccerRobot& defender =
          soccer_state.GetRobotByOurRobotIndex(tertiary_defender_indices_[i]);
      Pose2Df defensive_pose(0, 0, 0);
      defender.SetGoal(defensive_pose);
    }
  }

  defense_logger_.Pop();
}

bool DefenseEvaluator::GetBestTertiaryDefender(
    const WorldState& world_state, const SecondaryThreat& threat,
    const vector<DefenderTargetHelper>& defenders,
    unsigned int* defender_index) {
  float min_time = numeric_limits<float>::max();
  bool found_defender = false;
  ControlSequence2D control;
  for (unsigned int i = 0; i < defenders.size(); ++i) {
    const DefenderTargetHelper& defender = defenders[i];
    if (defender.is_intercepting_) continue;
    if (defender.is_assigned_) {
      if (!defender.could_be_improved_) {
        continue;
      } else if (threat.robot_id_ ==
                 secondary_threats_[defender.threat_index_].robot_id_) {
        *defender_index = i;
        return true;
      }
    }
    OurRobotIndex index = world_state.GetOurRobotIndex(defender.ssl_id_);
    Pose2Df defender_pose = world_state.GetOurRobotPosition(index).position;
    Vector2f defender_velocity =
        world_state.GetOurRobotPosition(index).velocity.translation;

    Vector2f ntoc_position;
    Vector2f ntoc_velocity;

    ntoc::TransformCoordinatesForNTOC(defender_pose, defender_velocity,
                                      threat.tertiary_target_, &ntoc_position,
                                      &ntoc_velocity);

    MotionModel model(kMaxRobotVelocity, kMaxRobotRotVel);
    float time_to_pose = NTOC2D(ntoc_position, ntoc_velocity, model, &control);

    if (time_to_pose < min_time) {
      if (defender.is_assigned_) {
        if (threat.score_ < secondary_threats_[defender.threat_index_].score_ +
                                kTertiaryCostFactor_ * time_to_pose) {
          min_time = time_to_pose;
          found_defender = true;
          *defender_index = i;
        }
      } else {
        min_time = time_to_pose;
        found_defender = true;
        *defender_index = i;
      }
    }
  }

  return found_defender;
}

bool DefenseEvaluator::GetBestSecondaryDefender(
    const WorldState& world_state, const SecondaryThreat& threat,
    const vector<DefenderTargetHelper>& defenders,
    unsigned int* defender_index) {
  float min_time = numeric_limits<float>::max();
  bool found_defender = false;
  ControlSequence2D control;
  for (unsigned int i = 0; i < defenders.size(); ++i) {
    const DefenderTargetHelper& defender = defenders[i];
    if (defender.is_intercepting_) continue;
    if (defender.is_assigned_) {
      if (!defender.could_be_improved_) {
        continue;
      } else if (threat.robot_id_ ==
                 secondary_threats_[defender.threat_index_].robot_id_) {
        *defender_index = i;
        return true;
      }
    }
    OurRobotIndex index = world_state.GetOurRobotIndex(defender.ssl_id_);
    Pose2Df defender_pose = world_state.GetOurRobotPosition(index).position;
    Vector2f defender_velocity =
        world_state.GetOurRobotPosition(index).velocity.translation;

    Vector2f ntoc_position;
    Vector2f ntoc_velocity;

    ntoc::TransformCoordinatesForNTOC(defender_pose, defender_velocity,
                                      threat.tertiary_target_, &ntoc_position,
                                      &ntoc_velocity);

    MotionModel model(kMaxRobotVelocity, kMaxRobotRotVel);
    float time_to_pose = NTOC2D(ntoc_position, ntoc_velocity, model, &control);

    if (time_to_pose < min_time) {
      if (defender.is_assigned_ && !defender.is_direct_) {
        if (threat.score_ < secondary_threats_[defender.threat_index_].score_ +
                                kSecondaryCostFactor_ * time_to_pose) {
          min_time = time_to_pose;
          found_defender = true;
          *defender_index = i;
        }
      } else {
        min_time = time_to_pose;
        found_defender = true;
        *defender_index = i;
      }
    }
  }

  return found_defender;
}

void DefenseEvaluator::UpdateOpenAngleWidths(const WorldState& world_state) {
  vector<Vector2f> blocker_positions;
  if (found_goalie_) {
    goalie_position_ =
        world_state.GetOurRobotPosition(goalie_index_).position.translation;
    blocker_positions.push_back(goalie_position_);
  }

  for (const auto& index : primary_defender_indices_) {
    Vector2f robot_position =
        world_state.GetOurRobotPosition(index).position.translation;
    blocker_positions.push_back(robot_position);
  }

  for (SecondaryThreat& threat : secondary_threats_) {
    threat.open_angle_width_ =
        GetWidestOpenAngle(threat.threat_position_, blocker_positions);
  }
}

void DefenseEvaluator::UpdatePreviouslyBlockedThreats(
    const WorldState& world_state, const SoccerState& soccer_state) {
  previous_primary_threats_.clear();
  for (const PrimaryThreat& threat : primary_threats_) {
    previous_primary_threats_.push_back(threat);
  }

  previous_blocked_secondary_.clear();
  for (const SecondaryThreat& threat : secondary_threats_) {
    if (threat.direct_blocked_ || threat.indirect_blocked_)
      previous_blocked_secondary_.push_back(threat);
  }
}

bool DefenseEvaluator::WasBlockingSecondary(const SSLVisionId id,
                                            unsigned int* threat_index,
                                            bool* was_direct) {
  for (unsigned int i = 0; i < previous_blocked_secondary_.size(); i++) {
    const SecondaryThreat& threat = previous_blocked_secondary_[i];
    *was_direct = threat.direct_blocking_id_ == id;
    if (*was_direct || threat.indirect_blocking_id_ == id) {
      return GetSecondaryThreat(threat.robot_id_, threat_index);
    }
  }
  return false;
}

bool DefenseEvaluator::GetSecondaryThreat(const SSLVisionId id,
                                          unsigned int* threat_index) {
  for (unsigned int i = 0; i < secondary_threats_.size(); i++) {
    const SecondaryThreat& threat = secondary_threats_[i];
    if (threat.robot_id_ == id) {
      *threat_index = i;
      return true;
    }
  }

  return false;
}

void DefenseEvaluator::UpdatePrimaryIntercept(const WorldState& world_state,
                                              const SoccerState& soccer_state) {
  static const bool kDebug = true;
  // Update Intercepting status
  const Vector2f current_ball_observation =
      world_state.GetBallPosition().observed_pose;

  if (kDebug) {
    defense_logger_.LogPrintPush("Update Primary Intercepting");
  }

  if (primary_intercepting_) {
    OurRobotIndex defender_index =
        world_state.GetOurRobotIndex(primary_intercept_id_);

    // Check to make sure this robot is still a secondary defender
    if (find(primary_defender_indices_.begin(), primary_defender_indices_.end(),
             defender_index) == primary_defender_indices_.end()) {
      primary_intercepting_ = false;
      primary_intercept_id_ = 42;
      primary_interception_count_ = 0;
      previous_intercept_pose_ = {0, 0, 0};
      return;
    }

    if (current_ball_observation.x() < -kFieldXMax ||
        PointInDefenseArea(current_ball_observation, -10, true)) {
      primary_intercepting_ = false;
      primary_intercept_id_ = 42;
      primary_interception_count_ = 0;
    }

    if (primary_interception_count_ > kInterceptHysterisis_) {
      primary_interception_count_ = false;
      primary_intercept_id_ = 42;
      primary_interception_count_ = 0;
    }
  }

  // Assign primary defender to block ball if it is headed for the goal
  const Vector2f& ball_position = world_state.GetBallPosition().position;
  const Vector2f& ball_velocity = world_state.GetBallPosition().velocity;

  bool fast_ball = SafeVectorNorm(ball_velocity) > kMinInterceptSpeed_;
  Pose2Df defender_pose(previous_intercept_pose_);

  if (kDebug) {
    if (fast_ball) {
      defense_logger_.LogPrint("Fast Ball: True");
    } else {
      defense_logger_.LogPrint("Fast Ball: False");
    }
  }

  if (fast_ball) {
    bool ball_in_defense =
        PointInDefenseArea(ball_position, kPrimaryDefenderInflation_, true);

    if (kDebug) {
      if (ball_in_defense) {
        defense_logger_.LogPrint("Ball In Defense: True");
      } else {
        defense_logger_.LogPrint("Ball In Defense: False");
      }
    }

    if (!ball_in_defense) {
      bool has_intersection = false;
      Vector2f intersect_position(0, 0);
      Vector2f ball_velocity_direction =
          field_dimensions::kFieldLength * ball_velocity / ball_velocity.norm();

      // TODO(slane): Cleanup this call
      for (const auto& obstacle : ObstacleFlag::GetOurDefenseArea()) {
        float distance;
        has_intersection = obstacle->FurthestFreePointOnLine(
            ball_position, ball_velocity_direction, &intersect_position,
            &distance, kPrimaryDefenderInflation_);
      }

      if (has_intersection) {
        defender_pose.translation = intersect_position;
        defender_pose.angle = Angle<float>(ball_position - intersect_position);
        primary_interception_count_ = 0;

        if (!primary_intercepting_) {
          float min_distance = numeric_limits<float>::max();
          OurRobotIndex min_index = 42;
          bool found_robot = false;
          // Select Primary Defender
          for (const OurRobotIndex& index : primary_defender_indices_) {
            Vector2f position =
                world_state.GetOurRobotPosition(index).position.translation;

            float intercept_distance =
                EuclideanDistance(intersect_position, position);
            float ball_distance = EuclideanDistance(position, ball_position);
            if (intercept_distance < min_distance &&
                ball_distance < kMaxInterceptDistance_) {
              min_index = index;
              min_distance = intercept_distance;
              found_robot = true;
            }
          }
          if (found_robot) {
            primary_intercepting_ = true;
            primary_intercept_id_ =
                world_state.GetOurRobotPosition(min_index).ssl_vision_id;
            primary_interception_count_ = 0;
            previous_intercept_pose_ = defender_pose;
          }
        }

        if (kDebug) {
          defense_logger_.LogPrint("Has Intersection: True");
        }
      } else if (kDebug) {
        defense_logger_.LogPrint("Has Intersection: False");
      }
    }
  }

  if (primary_intercepting_) {
    primary_interception_count_++;
    previous_intercept_pose_ = defender_pose;

    if (kDebug) {
      defense_logger_.AddCircle(defender_pose.translation, kRobotRadius, 1.0,
                                1.0, 0.0, 0.5);
      defense_logger_.LogPrint(
          "Intersection Pose: %f, %f, %f", defender_pose.translation.x(),
          defender_pose.translation.y(), defender_pose.angle);
    }
  }

  if (kDebug) {
    defense_logger_.Pop();
  }
}

void DefenseEvaluator::BoxBall(const WorldState& world_state,
                               const SoccerState& soccer_state) {
  static const bool kDebug = true;
  Vector2f ball_position = world_state.GetBallPosition().position;
  if (PointInDefenseArea(ball_position, kPrimaryDefenderInflation_, true)) {
    bool far_threshold = !PointInDefenseArea(
        ball_position, kBlockDistSolitary_ - kRobotRadius - kBallRadius, true);
    if (far_threshold) {
      Pose2Df defender_pose(0, 0, 0);

      // TODO(slane): Maybe get closest point on defense area rectangle
      Vector2f out_direction = ApproxDefenseAreaNorm(ball_position);

      defender_pose.translation =
          out_direction * kBlockDistSolitary_ + ball_position;
      defender_pose.angle = Angle<float>(out_direction);

      if (num_primary_defenders_ == 1) {
        PrimaryThreat threat;
        threat.defender_pose_ = defender_pose;
        previous_primary_threats_.push_back(threat);
        if (kDebug) {
          defense_logger_.AddCircle(defender_pose.translation, kRobotRadius,
                                    0.0, 1.0, 1.0, 0.5);
          defense_logger_.LogPrint(
              "Block Ball Pose: %f, %f, %f", defender_pose.translation.x(),
              defender_pose.translation.y(), defender_pose.angle);
        }
      } else if (num_primary_defenders_ > 1) {
        Vector2f perp = Perp(out_direction);
        Vector2f left_position = defender_pose.translation +
                                 perp * (kRobotRadius + kMinDistance_ / 2.0);
        Vector2f right_position = defender_pose.translation -
                                  perp * (kRobotRadius + kMinDistance_ / 2.0);

        if (kDebug) {
          defense_logger_.AddCircle(left_position, kRobotRadius, 0.0, 1.0, 1.0,
                                    0.5);
          defense_logger_.AddCircle(right_position, kRobotRadius, 0.0, 1.0, 1.0,
                                    0.5);
          defense_logger_.LogPrint("Left Block Position: %f, %f",
                                   left_position.x(), left_position.y());
          defense_logger_.LogPrint("Right Block Pose: %f, %f",
                                   right_position.x(), right_position.y());
        }
        PrimaryThreat threat;
        threat.defender_pose_ = Pose2Df(
            Angle(Vector2f(ball_position - left_position)), left_position);
        primary_threats_.push_back(threat);

        threat.defender_pose_ = Pose2Df(
            Angle(Vector2f(ball_position - right_position)), right_position);
        primary_threats_.push_back(threat);
      }
    }
  }
}

void DefenseEvaluator::BlockPrimaryThreats(const WorldState& world_state,
                                           const SoccerState& soccer_state) {
  static const bool kDebug = false;

  bool ball_in_defense = PointInDefenseArea(primary_threat_position_,
                                            kPrimaryDefenderMargin_, true);
  vector<AimOption> open_angles;
  vector<Vector2f> robot_positions;
  if (!ball_in_defense && !soccer_state.IsNormalPlay()) {
    if (num_primary_defenders_ == 1) {
      Vector2f defender_position;
      if (CanBlockFullAngleSingle(world_state, &defender_position)) {
        // First, see if we can block the open angle to the goal with just one
        // primary defender
        defense_logger_.LogPrint("One robot can block full angle");
        PrimaryThreat threat;
        threat.defender_pose_ = {
            Angle(Vector2f(primary_threat_position_ - defender_position)),
            defender_position};
        threat.notify_goalie_ = true;
        primary_threats_.push_back(threat);
        return;
      } else if (CanBlockFullAngleDouble(world_state, &defender_position)) {
        // Now, see if we can block the open angle to the goal with the goalie
        // and one primary defender
        defense_logger_.LogPrint("Two Robots can block full angle");
        PrimaryThreat threat;
        threat.defender_pose_ = {
            Angle(Vector2f(primary_threat_position_ - defender_position)),
            defender_position};
        threat.notify_goalie_ = true;
        primary_threats_.push_back(threat);
        return;
      }
    }

    // See if we can block the open angle to the goal with the goalie and two
    // primary defenders
    if (num_primary_defenders_ == 2) {
      defense_logger_.LogPrint("Checking if three can block");
      Vector2f left_position;
      Vector2f right_position;
      if (CanBlockFullAngleTriple(world_state, &left_position,
                                  &right_position)) {
        defense_logger_.LogPrint("Three Robots can block full angle");
        PrimaryThreat threat;
        threat.defender_pose_ = {
            Angle(Vector2f(primary_threat_position_ - left_position)),
            left_position};
        threat.notify_goalie_ = true;
        primary_threats_.push_back(threat);

        threat.defender_pose_ = {
            Angle(Vector2f(primary_threat_position_ - right_position)),
            right_position};
        primary_threats_.push_back(threat);
        return;
      }
    }
  }

  if (found_goalie_) {
    robot_positions.push_back(
        world_state.GetOurRobotPosition(goalie_index_).position.translation);
  }

  // Calculate aim options based on the current position of the goalie
  CalculateAimOptions(world_state.GetBallPosition().position, kOurGoalL,
                      kOurGoalR, kBallRadius, kRobotRadius, robot_positions,
                      &open_angles);

  std::sort(
      open_angles.begin(), open_angles.end(),
      [](AimOption a, AimOption b) { return b.angle_width < a.angle_width; });

  if (open_angles.empty() || primary_threat_position_.x() < -kFieldXMax ||
      ball_in_defense) {
    if (previous_primary_threats_.size() >= num_primary_defenders_) {
      for (const PrimaryThreat& threat : previous_primary_threats_) {
        PrimaryThreat new_threat(threat);
        new_threat.blocking_robot_id_ = 42;
        new_threat.is_blocked_ = false;
        new_threat.notify_goalie_ = false;
        new_threat.is_intercept_ = false;
        primary_threats_.push_back(threat);
      }
      return;
    }

    // The ball is in an odd position and we don't have previous positions,
    // block where you would if the ball is at the center of the field
    primary_threat_position_ = Vector2f(0, 0);

    CalculateAimOptions(world_state.GetBallPosition().position, kOurGoalL,
                        kOurGoalR, kBallRadius, kRobotRadius, robot_positions,
                        &open_angles);

    std::sort(
        open_angles.begin(), open_angles.end(),
        [](AimOption a, AimOption b) { return b.angle_width < a.angle_width; });
  }

  if (num_primary_defenders_ == 2 && open_angles.size() < 2) {
    AimOption option = open_angles[0];
    Vector2f goal_intercept = option.target_bisector;

    Vector2f defender_target = CalculateDefenderTarget(
        primary_threat_position_, goal_intercept, kPrimaryDefenderRatio_,
        kPrimaryDefenderMaxDistance_, kPrimaryDefenderInflation_);

    MovePrimaryThreat(goal_intercept, defender_target);
    return;
  }

  for (const AimOption& threat : open_angles) {
    Vector2f goal_intercept = threat.target_bisector;

    Vector2f defender_target = CalculateDefenderTarget(
        primary_threat_position_, goal_intercept, kPrimaryDefenderRatio_,
        kPrimaryDefenderMaxDistance_, kPrimaryDefenderInflation_);
    Vector2f threat_direction = (primary_threat_position_ - goal_intercept);
    float angle = Angle(threat_direction);
    PrimaryThreat target;
    target.defender_pose_ = Pose2Df(angle, defender_target);
    primary_threats_.push_back(target);
    if (kDebug) {
      defense_logger_.AddCircle(defender_target, kRobotRadius, 0.0, 0.0, 0.0,
                                0.5);
    }
  }
}

float DefenseEvaluator::CalculateSecondaryThreatScore(
    const Pose2Df& opponent_pose) {
  // Calculation taken from "CMDragons 2015: Coordinated Offense and
  // Defense of the SSL Champions"

  float cost = 0.0f;

  Vector2f threat_vector = primary_threat_position_ - opponent_pose.translation;
  float threat_dist = SafeVectorNorm(threat_vector);
  // Add pass time
  cost += threat_dist / kOpponentPassSpeed_;

  Vector2f goal_vector = kOurGoalCenter - opponent_pose.translation;

  float goal_dist = goal_vector.norm();

  float deflection_angle = acos(threat_vector.dot(goal_vector));
  deflection_angle = AngleMod(fabs(deflection_angle));

  float deflection_scaling = 1.0f;

  if (deflection_angle < kMinDeflectionAngle_) {
    // The angle is good for deflection
    deflection_scaling = 0.0f;
  } else if (deflection_angle < kMaxDeflectionAngle_) {
    // The angle is close to being good for deflection but they probably need to
    // move a bit, scale it
    deflection_scaling =
        (deflection_angle - kMinDeflectionAngle_) / kDeflectionAngleDiff_;
  }
  cost += kCatchTime_ * deflection_scaling;

  // Add shot time
  cost += goal_dist / kOpponentShootSpeed_;

  return cost;
}

Vector2f DefenseEvaluator::CalculateDefenderTarget(
    const Vector2f threat_position, const Vector2f goal_position, float ratio,
    float max_distance, float margin) {
  float distance = EuclideanDistance(threat_position, goal_position);
  float defender_distance = ratio * distance / (1.0f + ratio);
  Vector2f threat_direction = threat_position - goal_position;
  Vector2f target_position =
      defender_distance * threat_direction / SafeVectorNorm(threat_direction) +
      goal_position;

  if (EuclideanDistance(target_position, kOurGoalCenter) > max_distance) {
    float sq_distance;
    FurthestFreePointCircle(threat_position, goal_position, kOurGoalCenter,
                            max_distance, &sq_distance, &target_position);

    Vector2f direction = target_position - kOurGoalCenter;
    direction.normalize();
    target_position = direction * max_distance + kOurGoalCenter;
  }

  for (const auto& obstacle : ObstacleFlag::GetOurDefenseArea()) {
    Vector2f safe_target_position;
    float distance;
    obstacle->FurthestFreePointOnLine(threat_position, target_position,
                                      &safe_target_position, &distance, margin);
    target_position = safe_target_position;
  }

  return target_position;
}

float DefenseEvaluator::CalculatePassRecieveRisk(
    const Vector2f& ball_position, const Vector2f& ball_velocity,
    const float& ball_speed, const Vector2f& robot_position) {
  // This calculation is taken from
  // "Opponent-Driven Planning and Execution for Pass, Attack,
  // and Defense in a Multi-Robot Soccer Team" by Joydeep Biswas et. al.
  // AAMAS 2014

  Vector2f ball_opponent_vector = robot_position - ball_position;
  float opponent_dist = SafeVectorNorm(ball_opponent_vector);

  // cos of Angle between the ball velocity and the robot direction from the
  // ball
  float cos_theta =
      ball_opponent_vector.dot(ball_velocity) / (opponent_dist * ball_speed);

  return -opponent_dist / ball_speed * (1 + kRiskParam_ * (1 - cos_theta));
}

float DefenseEvaluator::GetWidestOpenAngle(
    const Vector2f& position, const vector<Vector2f>& blocker_positions) {
  vector<AimOption> threats;
  // Calculate aim options based on the input blocker positions
  CalculateAimOptions(position, kOurGoalL, kOurGoalR, kBallRadius, kRobotRadius,
                      blocker_positions, &threats);
  if (!threats.empty()) {
    std::sort(threats.begin(), threats.end(), [](AimOption a, AimOption b) {
      return b.angle_width < a.angle_width;
    });
    return threats[0].angle_width;
  } else {
    return false;
  }
}

void DefenseEvaluator::UpdatePrimaryDefenseInPlace(
    const WorldState& world_state) {
  primary_defender_in_position_ = false;
  for (const PrimaryThreat& threat : primary_threats_) {
    if (threat.is_blocked_) {
      OurRobotIndex index =
          world_state.GetOurRobotIndex(threat.blocking_robot_id_);
      Vector2f robot_position =
          world_state.GetOurRobotPosition(index).position.translation;
      float distance =
          EuclideanDistance(robot_position, threat.defender_pose_.translation);
      if (distance < kDefenderInPlace_) {
        primary_defender_in_position_ = true;
        return;
      }
    }
  }
}

bool DefenseEvaluator::CanBlockFullAngleSingle(const WorldState& world_state,
                                               Vector2f* defender_position) {
  static const bool kDebug = false;
  vector<AimOption> aim_options;
  vector<Vector2f> robot_positions;
  // First see if a single primary defender can block the whole angle
  CalculateAimOptions(primary_threat_position_, kOurGoalL, kOurGoalR,
                      kBallRadius, kRobotRadius, robot_positions, &aim_options);

  if (!aim_options.empty()) {
    int max_width_index = -1;
    float max_width = -1;
    int current_index = 0;
    for (const AimOption& option : aim_options) {
      if (option.angle_width > max_width) {
        max_width = option.angle_width;
        max_width_index = current_index;
      }
      current_index++;

      if (kDebug) {
        defense_logger_.AddLine(primary_threat_position_, option.target_l, 0, 0,
                                0, 1);
        defense_logger_.AddLine(primary_threat_position_, option.target_r, 0, 0,
                                0, 1);
      }
    }

    float angle_width = max_width;
    Vector2f target = aim_options[max_width_index].target_bisector;
    float full_block_distance_from_threat =
        kRobotRadius / tan(angle_width / 2.0);
    float threat_distance = EuclideanDistance(target, primary_threat_position_);

    float target_distance = threat_distance - full_block_distance_from_threat;
    *defender_position =
        target_distance *
            GetNormalizedOrZero(Vector2f(primary_threat_position_ - target)) +
        target;

    // Move target position out if needed
    for (const auto& obstacle : ObstacleFlag::GetOurDefenseArea()) {
      float distance;
      obstacle->FurthestFreePointOnLine(primary_threat_position_,
                                        *defender_position, defender_position,
                                        &distance, kPrimaryDefenderInflation_);
    }

    return target_distance <= kPrimaryDefenderMaxDistance_;
  }
  return false;
}

bool DefenseEvaluator::CanBlockFullAngleDouble(const WorldState& world_state,
                                               Vector2f* defender_position) {
  vector<AimOption> aim_options;
  vector<Vector2f> robot_positions;

  Vector2f goal_post;

  if (primary_threat_position_.y() > 0.0) {
    goal_post = kOurGoalR;
  } else {
    goal_post = kOurGoalL;
  }

  float sq_distance;
  FurthestFreePointCircle(primary_threat_position_, goal_post, kOurGoalCenter,
                          kPrimaryDefenderMaxDistance_ + kRobotRadius,
                          &sq_distance, defender_position);

  Vector2f direction = *defender_position - kOurGoalCenter;
  direction.normalize();
  *defender_position =
      direction * kPrimaryDefenderMaxDistance_ + kOurGoalCenter;

  robot_positions.push_back(*defender_position);

  CalculateAimOptions(primary_threat_position_, kOurGoalL, kOurGoalR,
                      kBallRadius, kRobotRadius, robot_positions, &aim_options);

  if (!aim_options.empty()) {
    int max_width_index = -1;
    float max_width = -1;
    int current_index = 0;
    for (const AimOption& option : aim_options) {
      if (option.angle_width > max_width) {
        max_width = option.angle_width;
        max_width_index = current_index;
      }
      current_index++;
      //       defense_logger_.AddLine(primary_threat_position_,
      //                               option.target_l,
      //                               1, 0, 0, 1);
      //       defense_logger_.AddLine(primary_threat_position_,
      //                               option.target_r,
      //                               0, 0, 1, 1);
    }
    Vector2f target = aim_options[max_width_index].target_bisector;

    Vector2f max_goalie_position;
    FurthestFreePointCircle(primary_threat_position_, target, kOurGoalCenter,
                            kGoalieMaxDistance, &sq_distance,
                            &max_goalie_position);
    float min_goalie_distance =
        EuclideanDistance(max_goalie_position, primary_threat_position_);
    float goalie_angular_width =
        2 * std::atan2(kRobotRadius, min_goalie_distance);

    return goalie_angular_width >= max_width;
  }
  return true;
}

bool DefenseEvaluator::CanBlockFullAngleTriple(const WorldState& world_state,
                                               Vector2f* left_position,
                                               Vector2f* right_position) {
  static const bool kDebug = false;
  vector<AimOption> aim_options;
  vector<Vector2f> robot_positions;

  float sq_distance;
  FurthestFreePointCircle(primary_threat_position_, kOurGoalL, kOurGoalCenter,
                          kPrimaryDefenderMaxDistance_ + kRobotRadius,
                          &sq_distance, left_position);

  Vector2f direction = *left_position - kOurGoalCenter;
  direction.normalize();
  *left_position = direction * kPrimaryDefenderMaxDistance_ + kOurGoalCenter;
  robot_positions.push_back(*left_position);

  FurthestFreePointCircle(primary_threat_position_, kOurGoalR, kOurGoalCenter,
                          kPrimaryDefenderMaxDistance_ + kRobotRadius,
                          &sq_distance, right_position);

  direction = *right_position - kOurGoalCenter;
  direction.normalize();
  *right_position = direction * kPrimaryDefenderMaxDistance_ + kOurGoalCenter;

  robot_positions.push_back(*right_position);

  CalculateAimOptions(primary_threat_position_, kOurGoalL, kOurGoalR,
                      kBallRadius, kRobotRadius, robot_positions, &aim_options);

  if (!aim_options.empty()) {
    int max_width_index = -1;
    float max_width = -1;
    int current_index = 0;
    for (const AimOption& option : aim_options) {
      if (option.angle_width > max_width) {
        max_width = option.angle_width;
        max_width_index = current_index;
      }
      current_index++;

      if (kDebug) {
        defense_logger_.AddLine(primary_threat_position_, option.target_l, 1, 0,
                                0, 1);
        defense_logger_.AddLine(primary_threat_position_, option.target_r, 0, 0,
                                1, 1);
      }
    }
    Vector2f target = aim_options[max_width_index].target_bisector;

    Vector2f max_goalie_position;
    FurthestFreePointCircle(primary_threat_position_, target, kOurGoalCenter,
                            kGoalieMaxDistance, &sq_distance,
                            &max_goalie_position);
    float min_goalie_distance =
        EuclideanDistance(max_goalie_position, primary_threat_position_);
    float goalie_angular_width =
        2 * std::atan2(kRobotRadius, min_goalie_distance);

    return goalie_angular_width >= max_width;
  } else {
    // Make sure the robots aren't in collision with each other
    if (EuclideanDistance(*left_position, *right_position) < kMinDistance_) {
      Vector2f midpoint = (*left_position + *right_position) / 2.0;
      Vector2f midpoint_dir = midpoint - primary_threat_position_;
      midpoint_dir.normalize();
      *left_position = midpoint - Perp(midpoint_dir) * kMinDistance_ / 2.0;
      *right_position = midpoint + Perp(midpoint_dir) * kMinDistance_ / 2.0;
    }

    return true;
  }
}

bool DefenseEvaluator::CouldIntercept(const Vector2f& ball_position,
                                      const Vector2f& ball_velocity,
                                      const Vector2f& robot_position,
                                      float* relative_percent) {
  Vector2f ball_to_robot_displace = robot_position - ball_position;
  ball_to_robot_displace.normalize();
  const float ball_speed = SafeVectorNorm(ball_velocity);

  const float relative_ball_vel = ball_to_robot_displace.dot(ball_velocity);
  *relative_percent = (relative_ball_vel / ball_speed) * 100;

  return ball_speed >= kMinInterceptSpeed_ &&
         *relative_percent >= kMinRelativePercent_;
}

void DefenseEvaluator::UpdateSecondaryIntercept(
    const WorldState& world_state, const state::SoccerState& soccer_state) {
  Vector2f ball_pos = world_state.GetBallPosition().position;
  Vector2f ball_vel = world_state.GetBallPosition().velocity;
  float max_percent = 0;

  if (BallMayEnterGoal(ball_pos, ball_vel, false) ||
      soccer_state.GetSharedState().GetPassShot()) {
    secondary_intercepting_ = false;
    secondary_intercept_id_ = 42;
    secondary_interception_count_ = 0;
    return;
  }

  if (soccer_state.GetRefereeState().IsBallPlacementThem() ||
      soccer_state.GetRefereeState().IsBallPlacementUs()) {
    secondary_intercepting_ = false;
    secondary_intercept_id_ = 42;
    secondary_interception_count_ = 0;
    return;
  }

  if (secondary_intercepting_) {
    OurRobotIndex defender_index =
        world_state.GetOurRobotIndex(secondary_intercept_id_);
    // Check to make sure this robot is still a secondary defender
    if (find(secondary_defender_indices_.begin(),
             secondary_defender_indices_.end(),
             defender_index) == secondary_defender_indices_.end()) {
      secondary_intercepting_ = false;
      secondary_intercept_id_ = 42;
      secondary_interception_count_ = 0;
      return;
    }

    Vector2f robot_pos =
        world_state.GetOurRobotPosition(defender_index).position.translation;

    if (CouldIntercept(ball_pos, ball_vel, robot_pos, &max_percent)) {
      secondary_interception_count_ = 0;
    } else {
      secondary_interception_count_++;
      if (secondary_interception_count_ > kInterceptHysterisis_) {
        secondary_intercepting_ = false;
        secondary_interception_count_ = 0;
        secondary_intercept_id_ = 42;
      }
    }
  } else {
    float ball_speed = SafeVectorNorm(ball_vel);
    if (ball_speed < kMinInterceptSpeed_) {
      return;
    }

    for (const OurRobotIndex& index : secondary_defender_indices_) {
      float current_percent;
      Vector2f robot_pos =
          world_state.GetOurRobotPosition(index).position.translation;
      bool could_intercept =
          CouldIntercept(ball_pos, ball_vel, robot_pos, &current_percent);

      float ball_distance = EuclideanDistance(ball_pos, robot_pos);
      if (could_intercept && current_percent > max_percent &&
          ball_distance < kMaxInterceptDistance_) {
        max_percent = current_percent;
        secondary_intercepting_ = true;
        secondary_interception_count_ = 0;
        secondary_intercept_id_ =
            world_state.GetOurRobotPosition(index).ssl_vision_id;
      }
    }
  }
}

void DefenseEvaluator::UpdateTertiaryIntercept(
    const WorldState& world_state, const state::SoccerState& soccer_state) {
  Vector2f ball_pos = world_state.GetBallPosition().position;
  Vector2f ball_vel = world_state.GetBallPosition().velocity;
  float max_percent = 0;

  if (BallMayEnterGoal(ball_pos, ball_vel, false) ||
      soccer_state.GetSharedState().GetPassShot()) {
    tertiary_intercepting_ = false;
    tertiary_intercept_id_ = 42;
    tertiary_interception_count_ = 0;
    return;
  }

  if (soccer_state.GetRefereeState().IsBallPlacementThem() ||
      soccer_state.GetRefereeState().IsBallPlacementUs()) {
    secondary_intercepting_ = false;
    secondary_intercept_id_ = 42;
    secondary_interception_count_ = 0;
    return;
  }

  if (tertiary_intercepting_) {
    OurRobotIndex defender_index =
        world_state.GetOurRobotIndex(tertiary_intercept_id_);
    // Check to make sure this robot is still a secondary defender
    if (find(tertiary_defender_indices_.begin(),
             tertiary_defender_indices_.end(),
             defender_index) == tertiary_defender_indices_.end()) {
      tertiary_intercepting_ = false;
      tertiary_intercept_id_ = 42;
      tertiary_interception_count_ = 0;
      return;
    }

    Vector2f robot_pos =
        world_state.GetOurRobotPosition(defender_index).position.translation;

    if (CouldIntercept(ball_pos, ball_vel, robot_pos, &max_percent)) {
      tertiary_interception_count_ = 0;
    } else {
      tertiary_interception_count_++;
      if (tertiary_interception_count_ > kInterceptHysterisis_) {
        tertiary_intercepting_ = false;
        tertiary_interception_count_ = 0;
        tertiary_intercept_id_ = 42;
      }
    }
  } else {
    float ball_speed = SafeVectorNorm(ball_vel);
    if (ball_speed < kMinInterceptSpeed_) {
      return;
    }

    for (const OurRobotIndex& index : tertiary_defender_indices_) {
      float current_percent;
      Vector2f robot_pos =
          world_state.GetOurRobotPosition(index).position.translation;
      bool could_intercept =
          CouldIntercept(ball_pos, ball_vel, robot_pos, &current_percent);

      float ball_distance = EuclideanDistance(ball_pos, robot_pos);
      if (could_intercept && current_percent > max_percent &&
          ball_distance < kMaxInterceptDistance_) {
        max_percent = current_percent;
        tertiary_intercepting_ = true;
        tertiary_interception_count_ = 0;
        tertiary_intercept_id_ =
            world_state.GetOurRobotPosition(index).ssl_vision_id;
      }
    }
  }
}

void DefenseEvaluator::MovePrimaryThreat(const Vector2f& goal_intercept,
                                         const Vector2f& defender_target) {
  static const bool kDebug = false;
  Vector2f threat_direction = (primary_threat_position_ - goal_intercept);
  threat_direction.normalize();
  Vector2f perp = Perp(threat_direction);
  Vector2f left_target =
      defender_target + perp * (kMinDistance_ / 2.0 + kRobotRadius);
  Vector2f right_target =
      defender_target - perp * (kMinDistance_ / 2.0 + kRobotRadius);

  PrimaryThreat target;
  target.defender_pose_ = Pose2Df(
      Angle(Vector2f(primary_threat_position_ - left_target)), left_target);
  primary_threats_.push_back(target);

  target.defender_pose_ = Pose2Df(
      Angle(Vector2f(primary_threat_position_ - right_target)), right_target);
  primary_threats_.push_back(target);

  if (kDebug) {
    defense_logger_.AddCircle(left_target, kRobotRadius, 0.0, 0.0, 0.0, 0.5);
    defense_logger_.AddCircle(right_target, kRobotRadius, 0.0, 0.0, 0.0, 0.5);
  }
}

Eigen::Vector2f DefenseEvaluator::ApproxDefenseAreaNorm(const Vector2f& point) {
  Vector2f norm_vector(1, 0);
  if (point.y() > kDefenseStretch) {
    float distance;
    Vector2f projected_point;
    ProjectPointOntoLineSegment(
        point, Vector2f(-kFieldXMax, kDefenseStretch),
        Vector2f(-kFieldXMax + kFieldXMax, kDefenseStretch), &projected_point,
        &distance);
    norm_vector = point - projected_point;

    // Ensure the vector is still pointing in the right direction even if it is
    // inside the defense area
    norm_vector.x() = fabs(norm_vector.x());
    norm_vector.y() = fabs(norm_vector.y());

    norm_vector.normalize();
  } else if (point.y() < -kDefenseStretch) {
    float distance;
    Vector2f projected_point;
    ProjectPointOntoLineSegment(
        point, Vector2f(-kFieldXMax, -kDefenseStretch),
        Vector2f(-kFieldXMax + kFieldXMax, -kDefenseStretch), &projected_point,
        &distance);
    norm_vector = point - projected_point;

    // Ensure the vector is still pointing in the right direction even if it is
    // inside the defense area
    norm_vector.x() = fabs(norm_vector.x());
    norm_vector.y() = -fabs(norm_vector.y());

    norm_vector.normalize();
  }

  // If it is not above or below, the norm is just straight forward
  return norm_vector;
}

// Initializations
logger::Logger DefenseEvaluator::defense_logger_;
Vector2f DefenseEvaluator::primary_threat_position_(0, 0);
Vector2f DefenseEvaluator::goalie_position_(0, 0);
bool DefenseEvaluator::found_goalie_ = false;

unsigned int DefenseEvaluator::num_primary_defenders_ = 0;
unsigned int DefenseEvaluator::num_secondary_defenders_ = 0;
unsigned int DefenseEvaluator::num_tertiary_defenders_ = 0;

vector<PrimaryThreat> DefenseEvaluator::primary_threats_;
vector<PrimaryThreat> DefenseEvaluator::previous_primary_threats_;

vector<SecondaryThreat> DefenseEvaluator::secondary_threats_;

vector<SecondaryThreat> DefenseEvaluator::previous_blocked_secondary_;

bool DefenseEvaluator::primary_defender_in_position_ = false;
bool DefenseEvaluator::primary_threat_is_robot_ = false;
SSLVisionId DefenseEvaluator::primary_threat_id_ = 42;

OurRobotIndex DefenseEvaluator::goalie_index_ = 42;
vector<OurRobotIndex> DefenseEvaluator::primary_defender_indices_;
vector<OurRobotIndex> DefenseEvaluator::secondary_defender_indices_;
vector<OurRobotIndex> DefenseEvaluator::tertiary_defender_indices_;

bool DefenseEvaluator::primary_intercepting_ = false;
int DefenseEvaluator::primary_interception_count_ = 0;
SSLVisionId DefenseEvaluator::primary_intercept_id_ = 42;
Pose2Df DefenseEvaluator::previous_intercept_pose_(0, 0, 0);

bool DefenseEvaluator::secondary_intercepting_ = false;
int DefenseEvaluator::secondary_interception_count_ = 0;
SSLVisionId DefenseEvaluator::secondary_intercept_id_ = 42;

bool DefenseEvaluator::tertiary_intercepting_ = false;
int DefenseEvaluator::tertiary_interception_count_ = 0;
SSLVisionId DefenseEvaluator::tertiary_intercept_id_ = 42;
}  // namespace defense
