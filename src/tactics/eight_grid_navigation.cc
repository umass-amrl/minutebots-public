// Copyright 2017 - 2019 kvedder@umass.edu
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

#include "tactics/eight_grid_navigation.h"

#include <limits>

#include "logging/navigation_logger.h"
#include "math/geometry.h"
#include "navigation/navigation_util.h"
#include "navigation/production/eight_grid_common_utils.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "tactics/ntoc_controller.h"

namespace tactics {

static constexpr float kDistanceFromCurrentPointToTargetNext = 200.0f;
static constexpr float kMinimumSwitchingCost = 40.0f;

using pose_2d::Pose2Df;
using state::SharedRobotState;

#define DEFAULT_OBSTACLES                    \
  (obstacle::ObstacleFlag::GetAllExceptTeam( \
      world_state_, *soccer_state_, our_robot_index_, our_robot_index_))

EightGridNavigation::EightGridNavigation(const state::WorldState& world_state,
                                         TacticArray* tactic_list,
                                         state::SharedState* shared_state,
                                         OurRobotIndex our_robot_index_,
                                         state::SoccerState* soccer_state,
                                         const bool is_goalie)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index_,
             soccer_state),
      eight_grid_(soccer_state->GetStaticCollisionGrid(),
                  soccer_state->GetDynamicCollisionGrid(),
                  kEightGridSquareSize),
      goal_(0, 0, 0),
      obstacles_(),
      margin_(),
      previous_plan_(),
      previous_plan_cost_(std::numeric_limits<float>::max()),
      timing_file_(),
      location_threshold_squared_(),
      linear_velocity_threshold_squared_(),
      angular_threshold_(),
      angular_velocity_threshold_() {
  Init();
}

EightGridNavigation::~EightGridNavigation() {
  if (timing_file_.is_open()) {
    timing_file_.flush();
    timing_file_.close();
  }
}

void EightGridNavigation::Init() {
  margin_.ResetToDefault();
  obstacles_ = DEFAULT_OBSTACLES;
  location_threshold_squared_ = Sq(kNavigationLocationThreshold);
  angular_threshold_ = kNavigationAngularThreshold;
  linear_velocity_threshold_squared_ = Sq(kNavigationLinearVelocityThreshold);
  angular_velocity_threshold_ = kNavigationAngularVelocityThreshold;
}

void EightGridNavigation::Reset() {}

void EightGridNavigation::SetGoal(const pose_2d::Pose2Df& pose) {
  goal_ = pose;
}

void EightGridNavigation::SetObstacles(
    const obstacle::ObstacleFlag& obstacles) {
  obstacles_ = obstacles;
}

void EightGridNavigation::SetAngularThreshold(const float angular_threshold) {
  angular_threshold_ = Sq(angular_threshold);
}

void EightGridNavigation::SetAngularVelocityThreshold(
    const float angular_velocity_threshold) {
  angular_velocity_threshold_ = Sq(angular_velocity_threshold);
}

void EightGridNavigation::SetLinearVelocityThreshold(
    const float linear_velocity_threshold) {
  linear_velocity_threshold_squared_ = Sq(linear_velocity_threshold);
}

void EightGridNavigation::SetLocationThreshold(const float location_threshold) {
  location_threshold_squared_ = Sq(location_threshold);
}

bool EightGridNavigation::FixedStartedInCollision(
    logger::Logger* robot_logger, const Eigen::Vector2f& current_position,
    Tactic* ntoc_controller) const {
  static constexpr float kFreePointBumpAmount = 40.0f;
  for (const auto* obstacle : obstacles_) {
    if (obstacle->PointCollision(
            current_position, margin_.GetMargin(obstacle->GetType()) - 0.01f)) {
      const auto free_point = obstacle->NearestFreePoint(
          current_position, margin_.GetMargin(obstacle->GetType()) + kEpsilon);
      const Eigen::Vector2f delta = free_point - current_position;
      const Eigen::Vector2f bump_delta =
          geometry::GetNormalizedOrZero(delta) * kFreePointBumpAmount;
      const Eigen::Vector2f final_point = current_position + delta + bump_delta;
      robot_logger->LogPrint("Started in collision, plowing to free space.");
      ntoc_controller->SetGoal({goal_.angle, final_point});
      ntoc_controller->Run();
      return true;
    }
  }
  return false;
}

void EightGridNavigation::HandleNoPathFound(
    logger::Logger* robot_logger, const Eigen::Vector2f& current_position,
    Tactic* ntoc_controller) const {
  robot_logger->LogPrint("No path found!");
  const Eigen::Vector2f& goal_position = goal_.translation;

  Vector2f min_free_point(0, 0);
  float min_distance_from_start =
      field_dimensions::kFieldLength * field_dimensions::kFieldWidth;

  Vector2f free_point(0, 0);
  float distance_from_start = 0;
  for (const auto* obstacle : obstacles_) {
    if (obstacle->FurthestFreePointOnLine(
            current_position, goal_position, &free_point, &distance_from_start,
            margin_.GetMargin(obstacle->GetType()) + kEpsilon)) {
      if (distance_from_start < min_distance_from_start) {
        min_free_point = free_point;
      }
    }
  }
  ntoc_controller->SetGoal({goal_.angle, min_free_point});
  ntoc_controller->Run();
}

bool EightGridNavigation::IsStraightPathCollisionFree(
    const Eigen::Vector2f& start, const Eigen::Vector2f& goal) const {
  for (const auto* obstacle : obstacles_) {
    if (obstacle->LineCollision(
            start, goal, margin_.GetMargin(obstacle->GetType()) - 1.0f)) {
      return false;
    }
  }
  return true;
}

Eigen::Vector2f EightGridNavigation::PickNTOCWaypoint(
    logger::Logger* robot_logger, const std::vector<Eigen::Vector2f>& path,
    const Eigen::Vector2f& current_position) const {
  constexpr bool kDebug = false;
  NP_CHECK_MSG(!path.empty(), "Cannot pick a waypoint on an empty path");

  if (kDebug) {
    robot_logger->LogPrint("Smoothed path");
    robot_logger->Push();
    for (size_t i = 0; i < path.size(); ++i) {
      const auto& p = path[i];
      robot_logger->LogPrint("%d: (%f, %f)", i, p.x(), p.y());
    }
    robot_logger->Pop();

    robot_logger->LogPrint("Path selection");
    robot_logger->Push();
  }
  size_t last_know_good_index = 0;
  for (size_t next_proposed_index = 0; next_proposed_index < path.size();
       ++next_proposed_index) {
    const Eigen::Vector2f& next_proposed_position = path[next_proposed_index];
    if (IsStraightPathCollisionFree(current_position, next_proposed_position)) {
      if (kDebug) {
        robot_logger->LogPrint("Accepted index %d", next_proposed_index);
        if (next_proposed_index + 1 == path.size()) {
          robot_logger->AddLine(current_position, path[next_proposed_index], 0,
                                1, 0, 1);
        }
      }
      last_know_good_index = next_proposed_index;
    } else {
      if (kDebug) {
        robot_logger->LogPrint("Rejected index %d", next_proposed_index);
        if (next_proposed_index + 1 == path.size()) {
          robot_logger->AddLine(current_position, path[next_proposed_index], 1,
                                0, 0, 1);
        }
      }
    }
  }
  if (kDebug) {
    robot_logger->Pop();
    robot_logger->LogPrint("Settled upon %d", last_know_good_index);
  }
  NP_CHECK(last_know_good_index < path.size());

  const Eigen::Vector2f& waypoint = path[last_know_good_index];
  if (last_know_good_index == path.size() - 1) {
    return waypoint;
  }

  // If the waypoint is too close, project it further ahead so that NTOC
  // "overshoots", thereby maintaining speed.
  const Eigen::Vector2f delta = waypoint - current_position;
  if (delta.squaredNorm() <= Sq(kDistanceFromCurrentPointToTargetNext)) {
    robot_logger->LogPrint("Extending waypoint further in front");
    return geometry::GetNormalizedOrZero(delta) *
               kDistanceFromCurrentPointToTargetNext +
           current_position;
  }
  return waypoint;
}

bool EightGridNavigation::PlanCollisionFree(
    const std::vector<Vector2f>& plan) const {
  const auto& static_cg = soccer_state_->GetStaticCollisionGrid();
  const auto& dynamic_cg = soccer_state_->GetDynamicCollisionGrid();
  for (const Vector2f& e : plan) {
    const auto grid_e =
        navigation::production::eight_grid::util::FreeSpaceToGridVertex(
            e, kEightGridSquareSize);
    if (static_cg.IsColliding(grid_e, obstacles_) ||
        dynamic_cg.IsColliding(grid_e, obstacles_)) {
      return false;
    }
  }
  return true;
}

float ComputePlanCost(const std::vector<Vector2f>& plan) {
  if (plan.empty()) {
    return std::numeric_limits<float>::max();
  }

  float cost = 0;
  for (size_t i = 0; i < plan.size() - 1; ++i) {
    cost += (plan[i + 1] - plan[i]).norm();
  }
  return cost;
}

bool GoalsDoNotMatch(const std::vector<Vector2f>& old_plan,
                     const std::vector<Vector2f>& new_plan,
                     const float location_threshold_squared) {
  if (old_plan.empty() || new_plan.empty()) {
    return false;
  }

  return (old_plan[old_plan.size() - 1] - new_plan[new_plan.size() - 1])
             .squaredNorm() > location_threshold_squared;
}

bool CostDecreaseWorthSwitching(const float old_cost, const float new_cost) {
  return (old_cost - new_cost) >= kMinimumSwitchingCost;
}

inline const char* const BoolToString(const bool b) {
  return b ? "true" : "false";
}

std::vector<Vector2f> GetShortenedPlan(const std::vector<Vector2f>& path) {
  if (path.size() > 2) {
    return {path.begin(), path.end() - 1};
  }
  return path;
}

std::vector<Vector2f> EightGridNavigation::DecideToUseOldOrNewPlan(
    logger::Logger* robot_logger, const std::vector<Vector2f>& new_plan) {
  if (new_plan.empty()) {
    return previous_plan_;
  }

  if (!kProduction && new_plan.size() > 2) {
    NP_CHECK(PlanCollisionFree(GetShortenedPlan(new_plan)));
  }
  const float new_plan_cost = ComputePlanCost(new_plan);
  const bool cost_worth =
      CostDecreaseWorthSwitching(previous_plan_cost_, new_plan_cost);
  const bool goals_no_match =
      GoalsDoNotMatch(previous_plan_, new_plan, location_threshold_squared_);
  const bool new_should_replace_old =
      (previous_plan_.size() == 2) || cost_worth || goals_no_match;
  const bool old_collision_free =
      PlanCollisionFree(GetShortenedPlan(previous_plan_));
  if (new_should_replace_old || !old_collision_free) {
    robot_logger->LogPrint(
        "Using fresh eight grid plan; "
        "new better: %s (Decrease: %s, Goals No Match: %s)"
        " old still collision free: %s",
        BoolToString(new_should_replace_old), BoolToString(cost_worth),
        BoolToString(goals_no_match), BoolToString(old_collision_free));
    previous_plan_ = new_plan;
    previous_plan_cost_ = new_plan_cost;
  } else {
    robot_logger->LogPrint("Reusing old eight grid plan");
  }

  return previous_plan_;
}

const Eigen::Vector2f KeepGoalInsideField(const Eigen::Vector2f& start,
                                          const Eigen::Vector2f& goal,
                                          const obstacle::SafetyMargin& margin,
                                          logger::Logger* logger) {
  logger->LogPrint("Project To Safety:");
  logger->Push();
  const auto result = navigation::ProjectToSafety(
      goal, start, margin.GetMargin(obstacle::ObstacleType::STATIC) + kEpsilon,
      logger);
  logger->Pop();
  if (result != goal) {
    logger->LogPrint("Projecting goal to safety, from (%f, %f) to (%f, %f)",
                     goal.x(), goal.y(), result.x(), result.y());
  }

  return result;
}

bool EightGridNavigation::SanitizeGoal(logger::Logger* robot_logger) {
  const Eigen::Vector2f& current_position =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;

  bool goal_aborted = false;
  bool goal_in_collision = navigation::ClosestPointInFreeSpace(
      obstacles_, margin_, &(goal_.translation), &goal_aborted, robot_logger);

  if (goal_in_collision) {
    robot_logger->LogPrint("Sanitized goal to (%f, %f, %f)", goal_.angle,
                           goal_.translation.x(), goal_.translation.y());
  }

  if (goal_aborted) {
    navigation::OneShotMode(obstacles_, margin_, current_position,
                            goal_.translation, &goal_.translation);
  }
  VerifyGoalCollisionFree();
  robot_logger->AddLine(goal_.translation - Vector2f(10, 10),
                        goal_.translation + Vector2f(10, 10), 0, 0, 0, 1);
  robot_logger->AddLine(goal_.translation - Vector2f(10, -10),
                        goal_.translation + Vector2f(10, -10), 0, 0, 0, 1);
  robot_logger->AddCircle(goal_.translation, 10, 0, 0, 0, 1);
  robot_logger->AddCircle(goal_.translation, 15, 0, 0, 0, 1);

  return goal_in_collision || goal_aborted;
}

void EightGridNavigation::VerifyGoalCollisionFree() const {
  if (!kProduction) {
    for (const auto* obstacle : obstacles_) {
      if (!obstacle->PointCollision(goal_.translation,
                                    margin_.GetMargin(obstacle->GetType())) -
          1.0f) {
        LOG(ERROR) << "Sanitization failed! Goal position "
                   << goal_.translation.x() << ", " << goal_.translation.y()
                   << " in collision with obstacle of type "
                   << obstacle->GetType() << " at position "
                   << obstacle->GetPose().translation.x() << ", "
                   << obstacle->GetPose().translation.y();
      }
    }
  }
}

void EightGridNavigation::DrawObstacles(logger::Logger* robot_logger) {
  for (const auto* obstacle : obstacles_) {
    if (obstacle->GetType() == obstacle::ObstacleType::ROBOT ||
        obstacle->GetType() == obstacle::ObstacleType::BALL) {
      robot_logger->AddCircle(
          obstacle->GetPose().translation,
          obstacle->GetRadius() + margin_.GetMargin(obstacle->GetType()), 1, 0,
          0, 1);
    } else {
      obstacle->DrawObstacle(robot_logger);
    }
  }
}

void EightGridNavigation::DrawPath(logger::Logger* robot_logger,
                                   const std::vector<Vector2f>& plan_path) {
  // Log path.
  for (const auto& plan_waypoint : plan_path) {
    robot_logger->AddCircle(plan_waypoint, 10, 1, 0, 0, 0.3);
  }
  robot_logger->AddCircle(plan_path[plan_path.size() - 1], 25, 0, 0, 1, 1);
}

bool EightGridNavigation::IsWithinTranslationThreshold() {
  const auto& position_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_);

  const Eigen::Vector2f& current_position =
      position_velocity.position.translation;
  const Eigen::Vector2f& current_velocity =
      position_velocity.velocity.translation;

  const bool location_correct =
      (current_position - goal_.translation).squaredNorm() <
      location_threshold_squared_;
  const bool velocity_correct =
      (current_velocity.squaredNorm() < linear_velocity_threshold_squared_);

  return location_correct && velocity_correct;
}

bool EightGridNavigation::IsWithinRotationThreshold() {
  const auto& position_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_);

  const float& current_angle = position_velocity.position.angle;
  const float current_angular_velocity = position_velocity.velocity.angle;
  const bool angular_location_correct =
      fabs(goal_.angle - current_angle) < angular_threshold_;
  const bool angular_velocity_correct =
      fabs(current_angular_velocity) < fabs(angular_velocity_threshold_);

  return (angular_location_correct && angular_velocity_correct);
}

bool EightGridNavigation::IsWithinCompletedThresholds() {
  return (IsWithinTranslationThreshold() && IsWithinRotationThreshold());
}

bool EightGridNavigation::IsComplete() { return IsWithinCompletedThresholds(); }

void EightGridNavigation::MoveRobotsToOurHalfDuringKickoffHack(
    const Eigen::Vector2f& current_position) {
  if (!(soccer_state_->GetRefereeState().IsPrepareKickoffUs() ||
        soccer_state_->GetRefereeState().IsPrepareKickoffThem()) ||
      (current_position.x() < -kDefaultSafetyMargin + 10.0)) {
    return;
  }

  obstacles_ =
      obstacles_ & ~obstacle::ObstacleFlag::GetKickoffOtherHalfObstacle();

  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  goal_.translation.x() = -kDefaultSafetyMargin;
  robot_logger->LogPrint("New Goal: %f, %f", goal_.translation.x(),
                         goal_.translation.y());
  robot_logger->LogPrint(
      "Move robots to our half hack; moving to kickoff centerline.");
}

void EightGridNavigation::Run() {
  static constexpr bool kDebug = true;
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrintPush("Eight Grid Navigation");

  if (kDebug) {
    DrawObstacles(robot_logger);
  }

  const Eigen::Vector2f& current_position =
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation;

  MoveRobotsToOurHalfDuringKickoffHack(current_position);

  if (IsComplete()) {
    Init();
    SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
    state->should_stop_angular = true;
    state->should_stop_linear = true;

    if (kLogNavigation) {
      navigation_logger::NavigationLogger::AddRobot(
          world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id, 0.0,
          true, current_position.x(), current_position.y(),
          world_state_.GetOurRobotPosition(our_robot_index_).position.angle,
          goal_.translation.x(), goal_.translation.y(), goal_.angle,
          std::vector<Vector2f>());
    }

    robot_logger->Pop();
    return;
  }

  NTOC_Controller* ntoc_controller =
      static_cast<NTOC_Controller*>((*tactic_list_)[TacticIndex::NTOC].get());
  if (FixedStartedInCollision(robot_logger, current_position,
                              ntoc_controller)) {
    Init();
    robot_logger->Pop();
    return;
  }

  Pose2Df original_goal = goal_;
  bool moved_goal = SanitizeGoal(robot_logger);

  const auto before_update_eight_grid_time = GetMonotonicTime();
  eight_grid_.Update(obstacles_, margin_, current_position, goal_.translation,
                     robot_logger);
  const auto plan = eight_grid_.Plan(robot_logger);
  const auto after_plan_eight_grid_time = GetMonotonicTime();

  if (kEightGridDumpTimings) {
    if (!timing_file_.is_open()) {
      timing_file_.open("production_eight_grid_timings_robot_index_" +
                            std::to_string(our_robot_index_) + ".txt",
                        std::ios::out);
    }
    timing_file_ << (after_plan_eight_grid_time -
                     before_update_eight_grid_time) *
                        1000.0
                 << '\n';
  }

  const auto& plan_path = DecideToUseOldOrNewPlan(robot_logger, plan.second);

  robot_logger->LogPrint("Eight Grid planning path to %f, %f",
                         goal_.translation.x(), goal_.translation.y());
  robot_logger->LogPrint(
      "Eight grid planning time: %f ms",
      (after_plan_eight_grid_time - before_update_eight_grid_time) * 1000.0f);

  if (!plan.first || plan_path.size() < 2) {
    if (kLogNavigation) {
      navigation_logger::NavigationLogger::AddRobot(
          world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id,
          (after_plan_eight_grid_time - before_update_eight_grid_time), false,
          current_position.x(), current_position.y(),
          world_state_.GetOurRobotPosition(our_robot_index_).position.angle,
          goal_.translation.x(), goal_.translation.y(), goal_.angle, plan_path);
    }
    HandleNoPathFound(robot_logger, current_position, ntoc_controller);

    Init();
    robot_logger->Pop();
    return;
  }

  DrawPath(robot_logger, plan_path);
  const Eigen::Vector2f ntoc_waypoint =
      PickNTOCWaypoint(robot_logger, plan_path, current_position);
  robot_logger->LogPrint("NTOC waypoint: %f, %f", ntoc_waypoint.x(),
                         ntoc_waypoint.y());

  // Reset
  Init();
  robot_logger->Pop();

  bool translation_complete = IsWithinTranslationThreshold();
  bool rotation_complete = IsWithinRotationThreshold();

  if (translation_complete) {
    ntoc_controller->SetTranslationComplete();
  }

  if (rotation_complete) {
    ntoc_controller->SetRotationComplete();
  }

  if (!translation_complete || !rotation_complete) {
    ntoc_controller->SetGoal({goal_.angle, ntoc_waypoint});
    ntoc_controller->Run();
  }
  ntoc_controller->Reset();

  if (kLogNavigation && (!translation_complete || !rotation_complete)) {
    navigation_logger::NavigationLogger::AddRobot(
        world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id,
        (after_plan_eight_grid_time - before_update_eight_grid_time),
        !moved_goal, current_position.x(), current_position.y(),
        world_state_.GetOurRobotPosition(our_robot_index_).position.angle,
        goal_.translation.x(), goal_.translation.y(), goal_.angle, plan_path);
  }
}

}  // namespace tactics
