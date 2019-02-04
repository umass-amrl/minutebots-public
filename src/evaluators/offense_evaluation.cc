// Copyright 2017-2018 joydeepb@cs.umass.edu, jaholtz@cs.umass.edu
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

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "evaluators/offense_evaluation.h"
#include "logging/logger.h"
#include "math/geometry.h"
#include "motion_control/ntoc_2d.h"
#include "navigation/navigation_util.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/robot_obstacle.h"
#include "obstacles/safety_margin.h"
#include "shared/common_includes.h"
#include "state/soccer_state.h"
#include "state/world_state.h"

using geometry::Angle;
using geometry::Cross;
using geometry::GetTangentPoints;
using geometry::LineLineIntersection;
using geometry::Perp;
using math_util::AngleDiff;
using math_util::SolveQuadratic;
using motion::MotionModel;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using Eigen::Vector2f;
using std::sort;
using std::vector;
using state::WorldRobot;
using state::SoccerState;
using state::WorldState;
using geometry::IsBetween;
using geometry::RayIntersect;
using geometry::EuclideanDistance;
using pose_2d::Pose2Df;
using obstacle::RobotObstacle;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using obstacle::Obstacle;
using navigation::CollisionFreePath;

namespace {
struct AimTangent {
  Vector2f tangent_dir;
  int value;

  AimTangent(const Vector2f& tangent_dir, int value)
      : tangent_dir(tangent_dir), value(value) {}

  bool operator<(const AimTangent& other) const {
    return (geometry::Cross(tangent_dir, other.tangent_dir) > 0.0);
  }
};

}  // namespace

namespace offense {

AimOption::AimOption(const Eigen::Vector2f& source,
                     const Eigen::Vector2f& target_left,
                     const Eigen::Vector2f& target_right, const float value)
    : source(source),
      target_l(target_left),
      target_r(target_right),
      target_center(0.5 * (target_left + target_right)),
      value(value) {
  const float angle_l = Angle<float>(target_left - source);
  const float angle_r = Angle<float>(target_right - source);
  angle_width = AngleDiff(angle_l, angle_r);
  angle_center = AngleMod(angle_r + 0.5 * angle_width + kPullRightAdjustment);

  Vector2f bisector_vector = Heading(angle_center);
  float squared_dist;
  Vector2f bisect_point;

  // Check to make sure the bisector actually intersects for safety
  if (RayIntersect(source, bisector_vector, target_left, target_right,
                   &squared_dist, &bisect_point)) {
    target_bisector = bisect_point;
  } else {
    target_bisector = target_center;
  }
}

void CalculateAimOptions(const Vector2f& source, Vector2f target_left,
                         Vector2f target_right, float shooting_radius,
                         float obstacle_radius,
                         const vector<Vector2f>& obstacles,
                         vector<AimOption>* aim_options) {
  // Shrink target by radius.
  // Tangent points to target_left.
  Vector2f target_l_t0(0, 0), target_l_t1(0, 0);
  GetTangentPoints(source, target_left, shooting_radius, &target_l_t0,
                   &target_l_t1);

  // Tangent points to target_right.
  Vector2f target_r_t0(0, 0), target_r_t1(0, 0);
  GetTangentPoints(source, target_right, shooting_radius, &target_r_t0,
                   &target_r_t1);

  if (Cross<float>(target_r_t1 - source, target_l_t0 - source) <= 0.0) {
    // The target is not large enough for the given radius.
    return;
  }

  // Apply the changes to shrink the target segment.
  target_right = LineLineIntersection<float>(source, target_r_t1, target_left,
                                             target_right);
  target_left = LineLineIntersection<float>(source, target_l_t0, target_left,
                                            target_right);

  vector<AimTangent> tangents;

  const Vector2f right_dir = target_right - source;
  const Vector2f left_dir = target_left - source;

  tangents.push_back(AimTangent(right_dir, -1));
  tangents.push_back(AimTangent(left_dir, 1));

  const Vector2f right_margin_normal = Perp(right_dir).normalized();
  const float right_margin_offset =
      right_margin_normal.dot(source) - obstacle_radius;

  const Vector2f left_margin_normal = (-Perp(left_dir)).normalized();
  const float left_margin_offset =
      left_margin_normal.dot(source) - obstacle_radius;

  const Vector2f target_margin_normal =
      Perp<float>(target_left - target_right).normalized();
  const float target_margin_offset =
      target_margin_normal.dot(target_right) - obstacle_radius;

  for (const auto& obstacle : obstacles) {
    if (obstacle.dot(right_margin_normal) - right_margin_offset < 0.0 ||
        obstacle.dot(left_margin_normal) - left_margin_offset < 0.0 ||
        obstacle.dot(target_margin_normal) - target_margin_offset < 0.0) {
      // Obstacle is not in the way of the target from the source.
      continue;
    }
    Vector2f tangent_l(0, 0), tangent_r(0, 0);
    GetTangentPoints(source, obstacle, shooting_radius + obstacle_radius,
                     &tangent_r, &tangent_l);
    float squared_dist_l;
    float squared_dist_r;
    Vector2f ray_direction_l = tangent_l - source;
    Vector2f ray_direction_l_neg = -ray_direction_l;
    Vector2f intersection_point;  // not used but necessary for ray intersect
    // call
    bool intersects_l = geometry::RayIntersect(
        source, ray_direction_l, target_right, target_left, &squared_dist_l,
        &intersection_point);

    Vector2f ray_direction_r = tangent_r - source;
    Vector2f ray_direction_r_neg = -ray_direction_r;

    bool intersects_r = geometry::RayIntersect(
        source, ray_direction_r, target_right, target_left, &squared_dist_r,
        &intersection_point);

    Vector2f line_intersect_l = geometry::LineLineIntersection<float>(
        tangent_l, source, target_left, target_right);
    Vector2f line_intersect_r = geometry::LineLineIntersection<float>(
        tangent_r, source, target_left, target_right);

    bool l_neg_intersection = false;
    bool r_neg_intersection = false;

    Vector2f l_intersect_dir = line_intersect_l - source;
    Vector2f l_diff =
        l_intersect_dir.normalized() - ray_direction_l_neg.normalized();
    Vector2f r_intersect_dir = line_intersect_r - source;
    Vector2f r_diff =
        r_intersect_dir.normalized() - ray_direction_r_neg.normalized();

    if (l_diff.norm() < .01) {
      l_neg_intersection = true;
    }
    if (r_diff.norm() < .01) {
      r_neg_intersection = true;
    }

    // Pushes back the target endpoints as aim tangents if the target line
    // segment is completely contained within the tangent pair. Otherwise ignore
    // the tangent pairs if none of them has an intersection with the
    // target line segment. If only one of them intersects the target line
    // segment, sets the non-intersecting tangent to the outer tangent from
    // the source to the appropriate target endpoint.
    // Contained within the two intersections
    if (IsBetween(line_intersect_l, line_intersect_r, target_left, 0.1f) &&
        IsBetween(line_intersect_l, line_intersect_r, target_right, 0.1f)) {
      tangent_l = target_l_t1;
      tangent_r = target_r_t0;
      tangents.push_back(AimTangent(tangent_l - source, -1));
      tangents.push_back(AimTangent(tangent_r - source, 1));
      // Left intersection is in negative direction (very wide tangent angle)
    } else if (l_neg_intersection && !r_neg_intersection) {
      tangent_l = target_l_t1;
      tangent_r = target_r_t0;
      tangents.push_back(AimTangent(tangent_l - source, -1));
      tangents.push_back(AimTangent(tangent_r - source, 1));
      // Right intersection is in negative direction (very wide tangent angle)
    } else if (!l_neg_intersection && r_neg_intersection) {
      tangent_l = target_l_t1;
      tangent_r = target_r_t0;
      tangents.push_back(AimTangent(tangent_l - source, -1));
      tangents.push_back(AimTangent(tangent_r - source, 1));
    } else if (intersects_r && intersects_l) {
      tangent_r = source + sqrt(squared_dist_r) * ray_direction_r.normalized();

      tangent_l = source + sqrt(squared_dist_l) * ray_direction_l.normalized();
      tangents.push_back(AimTangent(tangent_l - source, -1));
      tangents.push_back(AimTangent(tangent_r - source, 1));
    } else if (!intersects_l && intersects_r) {
      tangent_r = source + sqrt(squared_dist_r) * ray_direction_r.normalized();

      tangent_l = target_l_t1;
      tangents.push_back(AimTangent(tangent_l - source, -1));
      tangents.push_back(AimTangent(tangent_r - source, 1));
    } else if (intersects_l && !intersects_r) {
      tangent_r = target_r_t0;

      tangent_l = source + sqrt(squared_dist_l) * ray_direction_l.normalized();
      tangents.push_back(AimTangent(tangent_l - source, -1));
      tangents.push_back(AimTangent(tangent_r - source, 1));
    }
  }

  sort(tangents.begin(), tangents.end());

  int num_obstacles = 1;
  for (size_t i = 0; i < tangents.size(); ++i) {
    num_obstacles += tangents[i].value;
    if (num_obstacles == 0) {
      const Vector2f aim_right = source + tangents[i].tangent_dir;
      // There must necessarily be a left tangent, since tangents are
      // created in pairs.
      DCHECK_LE(i + 1, tangents.size());
      const Vector2f aim_left = source + tangents[i + 1].tangent_dir;
      aim_options->push_back(AimOption(source, aim_left, aim_right, 1.0));
    }
  }
}

float PassEvaluator(const state::WorldState& world_state,
                    SSLVisionId receiver_robot_id, SSLVisionId passing_robot_id,
                    Vector2f pass_receiving_pos, float* open_angle_center,
                    const zone::FieldZone& field_zone) {
  vector<Eigen::Vector2f> obstacle_robot_positions;
  for (const auto& robot : world_state.GetOurRobots()) {
    if (robot.confidence > kEpsilon &&
        robot.ssl_vision_id != receiver_robot_id &&
        robot.ssl_vision_id != passing_robot_id) {
      obstacle_robot_positions.push_back(robot.position.translation);
    }
  }
  for (const auto& robot : world_state.GetTheirRobots()) {
    obstacle_robot_positions.push_back(robot.position.translation);
  }

  // ****
  // Calculate the angle clearance for receiving a ball at the given position
  // TODO(srabiee): Set the source position based on the passing robot position
  //                as opposed to the ball position
  Vector2f source = world_state.GetBallPosition().position;
  Vector2f source_receiver_vec = pass_receiving_pos - source;
  float source_receiver_dist =
      sqrt(source_receiver_vec(0) * source_receiver_vec(0) +
           source_receiver_vec(1) * source_receiver_vec(1));

  float angle_source_receiver = Angle<float>(pass_receiving_pos - source);
  float clearance_angle = 0.0;

  float robot_mean_velocity = kMaxRobotVelocity / 2.0;  // mm/s
  float ball_mean_velocity = robot_mean_velocity * 3;   // mm/s
  float manuever_time = source_receiver_dist / ball_mean_velocity;
  float receive_region_radius = manuever_time * robot_mean_velocity;
  //   float receive_region_radius = 500.0;

  Vector2f targetl;
  Vector2f targetr;
  targetl(0) = pass_receiving_pos(0) -
               receive_region_radius * sin(angle_source_receiver);
  targetl(1) = pass_receiving_pos(1) +
               receive_region_radius * cos(angle_source_receiver);
  targetr(0) = pass_receiving_pos(0) +
               receive_region_radius * sin(angle_source_receiver);
  targetr(1) = pass_receiving_pos(1) -
               receive_region_radius * cos(angle_source_receiver);

  Vector2f new_targetl, new_targetr;
  field_zone.CropToZone(source, targetr, targetl, &new_targetr, &new_targetl,
                        kDefaultSafetyMargin);
  targetl = new_targetl;
  targetr = new_targetr;

  // Get all the aiming options toward the target line segment
  vector<AimOption> aim_options;
  CalculateAimOptions(source, targetl, targetr, kBallRadius, kRobotRadius,
                      obstacle_robot_positions, &aim_options);

  // Take the aim option with largest open angle
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
    }
    clearance_angle = max_width;
    *open_angle_center = aim_options[max_width_index].angle_center;
  } else {
    clearance_angle = 0.0;
    *open_angle_center = 0.0;
  }

  return clearance_angle;
}

float PassEvaluatorUnlimited(const state::WorldState& world_state,
                             SSLVisionId receiver_robot_id,
                             SSLVisionId passing_robot_id,
                             Vector2f pass_receiving_pos,
                             float* open_angle_center) {
  vector<Eigen::Vector2f> obstacle_robot_positions;
  //   for (const auto& robot : world_state.GetOurRobots()) {
  //     if (robot.ssl_vision_id != receiver_robot_id &&
  //         robot.ssl_vision_id != passing_robot_id) {
  //       obstacle_robot_positions.push_back(robot.position.translation);
  //     }
  //   }
  for (const auto& robot : world_state.GetTheirRobots()) {
    obstacle_robot_positions.push_back(robot.position.translation);
  }

  // ****
  // Calculate the angle clearance for receiving a ball at the given position
  // TODO(srabiee): Set the source position based on the passing robot position
  //                as opposed to the ball position
  Vector2f source = world_state.GetBallPosition().position;
  Vector2f source_receiver_vec = pass_receiving_pos - source;
  float source_receiver_dist =
      sqrt(source_receiver_vec(0) * source_receiver_vec(0) +
           source_receiver_vec(1) * source_receiver_vec(1));

  float angle_source_receiver = Angle<float>(pass_receiving_pos - source);
  bool blocked = false;
  float clearance_angle = M_PI;
  float min_clearance_r = M_PI;
  float min_clearance_l = M_PI;
  for (const auto& obstacle : obstacle_robot_positions) {
    if (blocked) {
      continue;
    }

    Vector2f source_obstacle_vec = obstacle - source;
    float dist_2_obstacle =
        sqrt(source_obstacle_vec(0) * source_obstacle_vec(0) +
             source_obstacle_vec(1) * source_obstacle_vec(1));

    // TODO(srabiee): maybe increase the distance threshold for skipping an
    // obstacle
    // Skip this obstacle if it is behind the receiver
    if (dist_2_obstacle > (source_receiver_dist + kRobotRadius)) {
      continue;
    }

    Vector2f tangent_l(0, 0), tangent_r(0, 0);
    GetTangentPoints(source, obstacle, kBallRadius + kRobotRadius, &tangent_r,
                     &tangent_l);

    float squared_dist;
    Vector2f ray_direction = pass_receiving_pos - source;
    Vector2f intersect_point;
    blocked =
        geometry::RayIntersect(source, ray_direction, tangent_r, tangent_l,
                               &squared_dist, &intersect_point);

    if (blocked) {
      min_clearance_r = 0.0;
      min_clearance_l = 0.0;
      continue;
    } else {
      float clearance_l = 0.0;
      float clearance_r = 0.0;
      float tangent_l_angle = Angle<float>(tangent_l - source);
      float tangent_r_angle = Angle<float>(tangent_r - source);
      float angle_diff = AngleDiff(tangent_r_angle, angle_source_receiver);
      if (angle_diff > 0.0) {
        clearance_l = angle_diff;

        if (clearance_l < min_clearance_l) {
          min_clearance_l = clearance_l;
        }
      } else {
        clearance_r = AngleDiff(angle_source_receiver, tangent_l_angle);

        if (clearance_r < min_clearance_r) {
          min_clearance_r = clearance_r;
        }
      }
    }
  }

  // TODO(srabiee): Maybe it would be better to change the clearance angle to
  // the open angle rather than the minimum clearance angle on right and left.

  //   clearance_angle = min_clearance_l + min_clearance_r;
  clearance_angle = min(min_clearance_l, min_clearance_r);
  // jaholtz: changed this to return the angle_source_receiver instead of
  // the center of the actual open angle until we've adjusted to use target
  // lines.
  *open_angle_center = angle_source_receiver;
  return clearance_angle;
}

float GetBestPassTarget(const state::WorldState& world_state,
                        state::SoccerState* soccer_state,
                        const OurRobotIndex& our_robot_index,
                        const SSLVisionId& passing_robot_id,
                        const zone::FieldZone& field_zone,
                        OurRobotIndex* recieving_robot) {
  const Vector2f our_robot_position =
      world_state.GetOurRobotPosition(our_robot_index).position.translation;
  const Vector2f bot_to_goal = kTheirGoalCenter - our_robot_position;
  float max_angle = 0;
  float best_pass_target = Angle(bot_to_goal);

  //   Default index is something higher than the actual number of robots
  //   Check if this is less than the number of robots
  *recieving_robot = 42;
  for (const state::SoccerRobot& robot : soccer_state->GetAllSoccerRobots()) {
    // Some robots are not for passing to.
    if ((robot.our_robot_index_ != our_robot_index && robot.enabled_) &&
        (robot.current_tactic_ != tactics::TacticIndex::GOALIE ||
         robot.current_tactic_ != tactics::TacticIndex::PRIMARY_DEFENDER)) {
      float pass_target;
      const auto& world_robot =
          world_state.GetOurRobotPosition(robot.our_robot_index_);
      Vector2f direction =
          world_robot.position.translation -
          world_state.GetOurRobotPosition(our_robot_index).position.translation;

      zone::FieldZone ours(zone::OUR_HALF);
      zone::FieldZone midfield(zone::MID_FIELD);
      const float distance = direction.norm();
      const bool far_enough = distance > kMinPassDistance;
      // This can be much more robust.
      // TODO(jaholtz) this should check if the robot is in line with our goal
      const bool not_self_score = world_robot.position.translation.x() > 0;
      if (far_enough && not_self_score &&
          !soccer_state->BallTheirPossession()) {
        const float pass_angle = PassEvaluator(
            world_state, world_robot.ssl_vision_id, passing_robot_id,
            world_robot.position.translation, &pass_target, field_zone);
        // Never pass back
        if (pass_angle > max_angle) {
          max_angle = pass_angle;
          best_pass_target = pass_target;
          *recieving_robot = robot.our_robot_index_;
        }
      }
    }
  }
  return best_pass_target;
}

// Finds the closest allied robot with sufficient space away from other team
// robots and then chips to that robot. If no ally has sufficient free
// space, kicks to the closest non-defender ally.
float GetBestChipTarget(const state::WorldState& world_state,
                        state::SoccerState* soccer_state,
                        const OurRobotIndex& our_robot_index) {
  float kDistanceFromOpponentsThreshold = 500.0;
  float best_distance = INFINITY;
  float best_pass_target = 0;
  float best_distance_open = INFINITY;
  float best_pass_target_open = 0;

  const auto& kicker = world_state.GetOurRobotPosition(our_robot_index);
  for (const state::SoccerRobot& robot : soccer_state->GetAllSoccerRobots()) {
    if ((robot.our_robot_index_ != our_robot_index && robot.enabled_) &&
        (robot.current_tactic_ != tactics::TacticIndex::GOALIE ||
         robot.current_tactic_ != tactics::TacticIndex::PRIMARY_DEFENDER)) {
      const auto& world_robot =
          world_state.GetOurRobotPosition(robot.our_robot_index_);

      Vector2f direction =
          world_robot.position.translation - kicker.position.translation;

      float distance = direction.norm();

      float pass_angle = Angle(direction);
      if (distance < best_distance) {
        best_distance = pass_angle;
        best_pass_target = distance;
      }
      bool open = true;
      for (auto villian : world_state.GetTheirRobots()) {
        const auto& vector_to_us =
            world_robot.position.translation - villian.position.translation;
        if (fabs(vector_to_us.norm()) < kDistanceFromOpponentsThreshold) {
          open = false;
        }
      }
      if (open) {
        if (distance < best_distance_open) {
          best_pass_target_open = pass_angle;
          best_distance_open = distance;
        }
      }
    }
  }
  if (best_distance_open < INFINITY) {
    return best_pass_target_open;
  }
  return best_pass_target;
}

float GetBestPassTargetKickoff(const state::WorldState& world_state,
                               state::SoccerState* soccer_state,
                               const OurRobotIndex& our_robot_index,
                               const SSLVisionId& passing_robot_id,
                               const zone::FieldZone& field_zone) {
  float max_angle = 0;
  float best_pass_target = 0;
  float kdistanceFromRobot = 1000;
  logger::Logger* logger =
      soccer_state->GetMutableRobotLoggerByOurRobotIndex(our_robot_index);
  for (const state::SoccerRobot& robot : soccer_state->GetAllSoccerRobots()) {
    if (robot.our_robot_index_ != our_robot_index &&
        robot.current_tactic_ != tactics::TacticIndex::GOALIE) {
      float pass_target;
      const auto& world_robot =
          world_state.GetOurRobotPosition(robot.our_robot_index_);
      Vector2f translation = world_robot.position.translation;
      translation.x() = translation.x() + kdistanceFromRobot;
      if (translation.x() > 0) {
        float pass_angle = PassEvaluator(world_state, world_robot.ssl_vision_id,
                                         passing_robot_id, translation,
                                         &pass_target, field_zone);
        logger->LogPrint("Robot id: %d Pass Angle: %f Target: %f pass_target",
                         world_robot.ssl_vision_id, pass_angle, pass_target);
        logger->AddCircle(translation, kRobotRadius, 1, 1, 1, 1);
        if (pass_angle > max_angle) {
          max_angle = pass_angle;
          best_pass_target = pass_target;
        }
      }
    }
  }
  return best_pass_target;
}

bool SetIsBallMotionObstacle(const state::WorldState& world_state,
                             const OurRobotIndex& our_robot_index,
                             obstacle::ObstacleFlag* obstacle_flag) {
  const auto& robot = world_state.GetOurRobotPosition(our_robot_index);
  const Vector2f ball_from_robot =
      world_state.GetBallPosition().position - robot.position.translation;
  const float angle_to_ball = Angle(ball_from_robot);
  const float angle_from_heading =
      AngleDiff(angle_to_ball, robot.position.angle);
  if (fabs(angle_from_heading) > kBallObstacleAngleRequirement) {
    obstacle::ObstacleFlag ball_flag;
    ball_flag = ball_flag.GetBall();
    *obstacle_flag = *obstacle_flag | ball_flag;
    return true;
  } else {
    obstacle::ObstacleFlag not_ball_flag;
    not_ball_flag = ~not_ball_flag.GetBall();
    *obstacle_flag = *obstacle_flag & not_ball_flag;
  }
  return false;
}

void GetTarget(const Vector2f& source, const WorldState& world_state,
               SoccerState* soccer_state, const OurRobotIndex& aiming_robot,
               const bool& pass_only, float* target_angle,
               OurRobotIndex* target_robot) {
  logger::Logger* robot_logger =
      soccer_state->GetMutableRobotLoggerByOurRobotIndex(aiming_robot);
  Pose2Df current_pose = world_state.GetOurRobotPosition(aiming_robot).position;
  OurRobotIndex receiving_robot = 42;
  vector<Vector2f> world_robot_positions;
  for (const auto& robot : world_state.GetOurRobots()) {
    if (world_state.GetOurRobotIndex(robot.ssl_vision_id) !=
        static_cast<int>(aiming_robot)) {
      world_robot_positions.push_back(robot.position.translation);
    }
  }
  int i = 0;
  for (const auto& robot : world_state.GetTheirRobots()) {
    ++i;
    world_robot_positions.push_back(robot.position.translation);
  }
  bool should_pass = false;
  if (!pass_only && source.x() > kGoalDistanceThreshold) {
    vector<AimOption> aim_options;
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, world_robot_positions, &aim_options);
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
      }
      *target_angle = aim_options[max_width_index].angle_center;
      robot_logger->LogPrint("Open Angle to Goal Width: %f", max_width);
      if (max_width < kGoalAngleThreshold) {
        should_pass = true;
      }
    } else {
      should_pass = true;
    }
  } else {
    should_pass = true;
  }
  // If no good aim option for shooting on the goal, pass the ball.
  if (should_pass) {
    zone::FieldZone field_zone(zone::FULL_FIELD);
    *target_angle = GetBestPassTarget(
        world_state, soccer_state, aiming_robot,
        world_state.GetOurRobotPosition(aiming_robot).ssl_vision_id, field_zone,
        &receiving_robot);
  }
  *target_robot = receiving_robot;
  //   // Drawing a line from the robot in the direction of its target.
  const Vector2f target_direction = Heading(*target_angle);
  const Vector2f target_end =
      current_pose.translation + (4000 * target_direction);
  robot_logger->AddLine(current_pose.translation, target_end, 1.0, 0.0, 1.0,
                        1.0);
}

bool GetOpenAngleGoal(const Vector2f& source, const WorldState& world_state,
                      SoccerState* soccer_state,
                      const OurRobotIndex& aiming_robot, float* target_angle) {
  logger::Logger* robot_logger =
      soccer_state->GetMutableRobotLoggerByOurRobotIndex(aiming_robot);
  Pose2Df current_pose = world_state.GetOurRobotPosition(aiming_robot).position;
  vector<Vector2f> world_robot_positions;
  for (const auto& robot : world_state.GetOurRobots()) {
    if (world_state.GetOurRobotIndex(robot.ssl_vision_id) !=
        static_cast<int>(aiming_robot)) {
      world_robot_positions.push_back(robot.position.translation);
    }
  }
  int i = 0;
  for (const auto& robot : world_state.GetTheirRobots()) {
    ++i;
    world_robot_positions.push_back(robot.position.translation);
  }
  // Must be close enough to the goal to try to shoot on the goal.
  if (source.x() > kGoalDistanceThreshold) {
    vector<AimOption> aim_options;
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, world_robot_positions, &aim_options);
    if (!aim_options.empty()) {
      int max_width_index = -1;
      float max_width = -1;
      int current_index = 0;
      // Find the best aim option
      for (const AimOption& option : aim_options) {
        if (option.angle_width > max_width) {
          max_width = option.angle_width;
          max_width_index = current_index;
        }
        current_index++;
      }
      *target_angle = aim_options[max_width_index].angle_center;
      robot_logger->LogPrint("Open Angle to Goal Width: %f", max_width);
      // Only kick at open angles greater than some limit.
      if (max_width < kGoalAngleThreshold) {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

float GetKickAngle(const Vector2f& ball_velocity, const Vector2f& robot_pose,
                   const float& kick_speed, logger::Logger* robot_logger,
                   const Vector2f& target_vector) {
  //   return Angle(target_vector);
  static const bool kDebug = false;
  const Vector2f final_velocity;
  const Vector2f neg_ball = -ball_velocity;
  float left = Angle(neg_ball);
  float target_angle = Angle(target_vector);
  const float angle_diff = target_angle - left;
  float right = target_angle;
  float least_error = 999;
  float best_angle = (left + right) / 2;
  Vector2f final_result;
  while (fabs(least_error) > kEpsilon && fabs((left - right)) > DegToRad(3.0)) {
    float angle = (left + right) / 2;
    best_angle = angle;
    const float damping = 0.1;
    const float reflection_scaled = 0.5;
    const Vector2f robot_heading = {cos(angle), sin(angle)};
    const Vector2f robot_perp = {-sin(angle), cos(angle)};

    const Vector2f tangential_velocity =
        (robot_perp.dot(damping * ball_velocity) * robot_perp);
    const Vector2f ball_reflect =
        ball_velocity -
        (2 * (ball_velocity.dot(robot_heading) * robot_heading));
    const Vector2f reflected_velocity = reflection_scaled * ball_reflect;
    const Vector2f kick_velocity = kick_speed * robot_heading;

    Vector2f resultant_vector =
        tangential_velocity + reflected_velocity + kick_velocity;
    resultant_vector.normalize();
    float result_angle = Angle(resultant_vector);
    least_error = target_angle - result_angle;
    final_result = resultant_vector;
    if (kDebug) {
      robot_logger->AddLine(robot_pose, robot_pose + 1000 * final_result, 0, 0,
                            1.0, 1.0);
    }
    static const float kStepSize = 1.0;
    if (angle_diff < 0) {
      if (least_error < 0) {
        left = angle + DegToRad(kStepSize);
      } else {
        right = angle - DegToRad(kStepSize);
      }
    } else {
      if (least_error > 0) {
        left = angle + DegToRad(kStepSize);
      } else {
        right = angle - DegToRad(kStepSize);
      }
    }
  }
  const Vector2f final_vector = Heading(best_angle);
  if (kDebug) {
    robot_logger->AddLine(robot_pose, robot_pose + 2000 * final_vector, 0, 1.0,
                          0, 0.8);
    robot_logger->AddLine(robot_pose, robot_pose + 2000 * target_vector, 1.0,
                          0.0, 1.0, 0.8);
  }
  return best_angle;
}

void PrintThresholds() {
  LOG(WARNING) << kThreshAngle;
  LOG(WARNING) << kThreshDistance;
  LOG(WARNING) << kThreshYVel;
  LOG(WARNING) << kThreshRelVel;
  LOG(WARNING) << kThreshAlign;
  LOG(WARNING) << kAngularVel;
}

float GetTargetEvaluated(const Vector2f& source, const WorldState& world_state,
                         SoccerState* soccer_state,
                         const OurRobotIndex& aiming_robot,
                         const bool& pass_only, bool* chip,
                         float* chip_distance, Vector2f* target_position,
                         float* target_angle, OurRobotIndex* target_robot,
                         OurRobotIndex* last_target) {
  bool should_pass = false;
  float final_score = 0;
  OurRobotIndex receiving_robot = kNoPassRobot;
  logger::Logger* robot_logger =
      soccer_state->GetMutableRobotLoggerByOurRobotIndex(aiming_robot);
  *chip = false;
  if (!pass_only) {
    should_pass = !GetOpenAngleGoal(source, world_state, soccer_state,
                                    aiming_robot, target_angle);
  } else {
    should_pass = true;
  }
  if (soccer_state->GetRobotByOurRobotIndex(aiming_robot).current_tactic_ ==
      tactics::INDIRECT_FREE_KICKER) {
    should_pass = true;
  }
  // Calculate the best pass target
  if (should_pass) {
    // Search the pass locations on our list and find one that can be reached
    // in time.
    const size_t our_robots_num = world_state.GetOurRobots().GetElementCount();
    float best_score = 5.0;
    for (size_t i = 0; i < our_robots_num; ++i) {
      if (i != aiming_robot) {
        const Pose2Df robot_pose = world_state.GetOurRobotPosition(i).position;
        const Pose2Df robot_vel = world_state.GetOurRobotPosition(i).velocity;
        const Vector2f receiving_pose = soccer_state->setup_positions_[i].first;
        const float receiver_time = GetNtocTime(robot_pose,
                                                robot_vel.translation,
                                                receiving_pose);
        const Pose2Df attacker_vel =
            world_state.GetOurRobotPosition(aiming_robot).velocity;
        const Pose2Df attacker_pose =
            world_state.GetOurRobotPosition(aiming_robot).position;
        const float attacker_time =
            GetNtocTime(attacker_pose, attacker_vel.translation, source);
        const float ball_arrival_time =
            GetBallTravelTime(kPassSpeed, source, receiving_pose);
        const float receiver_buffer_time =
            (attacker_time + ball_arrival_time) - receiver_time;

        const Vector2f target_vector = receiving_pose - source;
        robot_logger->LogPrint("Robot %u, Receive Buffer Time: %f Score: %f", i,
                               receiver_buffer_time,
                               soccer_state->setup_scores_[i]);
        if (receiver_buffer_time > kReceivableThresh) {
          if (soccer_state->setup_scores_[i] < best_score) {
            receiving_robot = i;
            *target_angle = Angle(target_vector);
            *target_position = receiving_pose;
            best_score = soccer_state->setup_scores_[i];
          }
        }
      }
    }
    // Fallback to old targeting function
    if (best_score == 5.0) {
      GetTarget(source, world_state, soccer_state, aiming_robot, pass_only,
                target_angle, &receiving_robot);
      best_score = 1.0;
    }
    *target_robot = receiving_robot;

    robot_logger->LogPrint("Last Target: %d", *last_target);
    if (*last_target != kNoPassRobot) {
      const float old_target_score = soccer_state->setup_scores_[*last_target];
      robot_logger->LogPrint("Last Score: %f", old_target_score);
      if (old_target_score - best_score < kPassChangeThreshold) {
        best_score = old_target_score;
        *target_robot = *last_target;
        *target_position = soccer_state->setup_positions_[*last_target].first;
        const Vector2f target_vector = *target_position - source;
        *target_angle = Angle(target_vector);
      }
    }
    *last_target = *target_robot;
    if (should_pass) {
      Vector2f receive_position;
      ObstacleFlag obstacles;
      if (receiving_robot != kNoPassRobot) {
        obstacles =
            ObstacleFlag::GetAllRobotsExceptTeam(*target_robot);
        obstacles = obstacles &
            ObstacleFlag::GetAllRobotsExceptTeam(aiming_robot);
        receive_position =
            world_state.GetOurRobotPosition(*target_robot).position.translation;
      } else {
        receive_position = kTheirGoalCenter;
        obstacles =
            ObstacleFlag::GetAllRobotsExceptTeam(aiming_robot);
      }
      if (best_score < 1.0) {
        receive_position = *target_position;
      }
      SafetyMargin safety_margin;

      std::pair<bool, const Obstacle*> is_collision_free_info =
          navigation::CollisionFreePathGetObstacle(obstacles, safety_margin,
                                                   source, receive_position);
      if (!is_collision_free_info.first) {
        *chip = true;
        const Vector2f obstacle_pose =
            is_collision_free_info.second->GetPose().translation;
        const Vector2f obstacle_distance = obstacle_pose - source;
        *chip_distance = fabs(obstacle_distance.norm()) + kChipKickPadding;
      }
    }
  }
  return final_score;
}

float GetTargetEvaluated(const Vector2f& source, const WorldState& world_state,
                         SoccerState* soccer_state,
                         const OurRobotIndex& aiming_robot,
                         const bool& pass_only, Vector2f* target_position,
                         float* target_angle, OurRobotIndex* target_robot,
                         OurRobotIndex* last_target) {
  bool chip = false;
  float chip_distance = 0;
  return GetTargetEvaluated(source, world_state, soccer_state, aiming_robot,
                            pass_only, &chip, &chip_distance, target_position,
                            target_angle, target_robot, last_target);
}

float GetFinalBallSpeed(const float& ball_speed, const Vector2f& ball_pose,
                        const Vector2f& final_pose) {
  const float time = GetBallTravelTime(ball_speed, ball_pose, final_pose);
  return ball_speed - kBallAcceleration * time;
}

float GetBallTravelTime(const float& ball_speed,
                        const Eigen::Vector2f& ball_pose,
                        const Eigen::Vector2f& final_pose) {
  // Uses simulation ball model
  float ball_arrival_time = 0;
  float ball_other_soln = 0;
  int ball_num_solns = 0;
  // s = ut + 0.5at^2
  //   const float friction_coefficient = 0.0005f;
  //   const float friction_decel = 9.81f * friction_coefficient * 1000;
  ball_num_solns = SolveQuadratic(-0.5f * kBallAcceleration, ball_speed,
                                  -(final_pose - ball_pose).norm(),
                                  &ball_other_soln, &ball_arrival_time);
  if (ball_num_solns == 0) {
    ball_arrival_time = -1.0f;  // is there a better constant for this
  } else if (ball_other_soln > 0) {
    ball_arrival_time = ball_other_soln;
  }
  return ball_arrival_time;
}

float GetNtocTime(const pose_2d::Pose2Df& start,
                  const Eigen::Vector2f& velocity,
                  const Eigen::Vector2f& goal) {
  MotionModel motion_model(kMaxRobotAcceleration, kMaxRobotVelocity);
  ControlSequence2D linear_control;

  Vector2f ntoc_position;
  Vector2f ntoc_velocity;

  ntoc::TransformCoordinatesForNTOC(start, velocity, goal, &ntoc_position,
                                    &ntoc_velocity);

  return (NTOC2D(ntoc_position, ntoc_velocity, motion_model, &linear_control));
}

float GetPoseCost(Vector2f ball, vector<unsigned int> robots_to_ignore,
                  Vector2f position, const WorldState& world_state) {
  float aggressive_cost = 0;
  return GetPoseCost(ball, robots_to_ignore, position, world_state,
                     &aggressive_cost);
}

// Sadegh's Cost function from support attacker
float GetPoseCost(Vector2f ball_pos, vector<unsigned int> robots_to_ignore,
                  Vector2f position, const WorldState& world_state,
                  float* aggressive_cost) {
  // Used to determine whether the support attacker should look at the goal
  // or the ball
  const float min_goal_open_angle = (1.0 / 180.0) * M_PI;
  // The maximum passing clearance angles used for saturating the angle
  const float passing_clearance_max = (20.0 / 180.0) * M_PI;
  // Used to prevent rapid switching when there exists no open angle
  // towards the goal.
  const float goal_open_angle_bias = (1.0 / 180.0) * M_PI;
  // Used to prevent the support attackers from camping close
  // to the primary offensive role
  //   const float kAttackerPadding = 15.0 * kRobotRadius;

  //   const float deflection_angle_max_threshold = (50.0 / 180.0) * M_PI;
  //   const float deflection_angle_min_threshold = (10.0 / 180.0) * M_PI;
  //   const float deflection_penalty_rate = 0.0002 / (1.0 / 180.0) * M_PI;

  // Initialization
  unsigned int attacker_index = robots_to_ignore[0];
  if (attacker_index == kNoPassRobot) {
    attacker_index = 0;
  }
  float cost = 1.0;
  float goal_angle_width = 0.0;
  float goal_angle = 0.0;
  const Pose2Df attacker_pose =
      world_state.GetOurRobotPosition(attacker_index).position;

  // Points should not be near the attacker
  const float attacker_dist =
      EuclideanDistance(attacker_pose.translation, position);

  if (attacker_dist < kMinPassDistance) {
    return 1.0;
  }
  // Points cannot be in the defense areas.
  static const float kDefenseAreaBuffer = 4 * kRobotRadius;
  obstacle::ObstacleFlag flags = obstacle::ObstacleFlag::GetDefenseAreas();
  for (auto obstacle : flags) {
    if (obstacle->PointCollision(position, kDefenseAreaBuffer)) {
      return 1.0;
    }
  }

  // Angle from the ball to the evaluated position
  float support_ball_angle = Angle<float>(ball_pos - position);

  // Decide which robots to treat as obstacles.
  size_t our_robots_num = world_state.GetOurRobots().GetElementCount();
  vector<Vector2f> obstacle_robot_positions;
  for (size_t k = 0; k < our_robots_num; k++) {
    if (k == attacker_index ||
        std::find(robots_to_ignore.begin(), robots_to_ignore.end(), k) !=
            robots_to_ignore.end()) {
      continue;
    } else {
      // Only treating robots greater than a set distance away from our robots
      // as obstacles.
      //       const Vector2f our_trans =
      //           world_state.GetOurRobotPosition(k).position.translation;
      //       const float dist = EuclideanDistance(our_trans, ball_pos);
      //       if (dist > kChippableDistance) {
      obstacle_robot_positions.push_back(
          world_state.GetOurRobotPosition(k).position.translation);
      //       }
    }
  }
  vector<Vector2f> aggressive_obstacles;
  for (const auto& robot : world_state.GetTheirRobots()) {
    // Only treating robots greater than a set distance away from our robots
    // as obstacles.
    const Vector2f enemy_trans = robot.position.translation;
    //     const float dist = EuclideanDistance(enemy_trans, ball_pos);
    const float goal_dist = EuclideanDistance(enemy_trans, kTheirGoalCenter);
    //     if (dist > kChippableDistance) {
    obstacle_robot_positions.push_back(robot.position.translation);
    //     }
    if (goal_dist < kPrimaryDefenseDistance) {
      aggressive_obstacles.push_back(robot.position.translation);
    }
  }

  // Get all the aiming options toward the goal
  vector<AimOption> aim_options;
  CalculateAimOptions(position, kTheirGoalL, kTheirGoalR, kBallRadius,
                      kRobotRadius, obstacle_robot_positions, &aim_options);

  // Take the aim option with largest open angle
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
    }
    goal_angle_width = max_width;
    goal_angle = aim_options[max_width_index].angle_center;
  } else {
    goal_angle_width = 0.0;
    goal_angle = 0.0;
  }

  // Do the same thing for the aggressive obstacles....
  // Get all the aiming options toward the goal
  CalculateAimOptions(position, kTheirGoalL, kTheirGoalR, kBallRadius,
                      kRobotRadius, aggressive_obstacles, &aim_options);
  float aggressive_goal_angle_width;
  // Take the aim option with largest open angle
  if (!aim_options.empty()) {
    float max_width = -1;
    int current_index = 0;
    for (const AimOption& option : aim_options) {
      if (option.angle_width > max_width) {
        max_width = option.angle_width;
      }
      current_index++;
    }
    aggressive_goal_angle_width = max_width;
  } else {
    aggressive_goal_angle_width = 0.0;
  }
  // -----------------------------------------------------------

  pose_2d::Pose2Df proposed_pose;
  // Returning the proposed pose of the robot given it is going to
  // receive the ball at the given position.
  if (goal_angle_width > min_goal_open_angle) {
    proposed_pose.angle = goal_angle;
  } else {
    proposed_pose.angle = support_ball_angle;
  }
  proposed_pose.translation.x() = position(0);
  proposed_pose.translation.y() = position(1);

  // Calculate the angle clearance for receiving a ball at the given position
  float clearance_angle = 0.0;
  SSLVisionId attacker_id =
      world_state.GetOurRobotPosition(attacker_index).ssl_vision_id;
  float pass_open_angle_center;
  clearance_angle = offense::PassEvaluatorUnlimited(
      world_state, attacker_id, attacker_id, position, &pass_open_angle_center);

  // Saturate the passing clearance angle
  if (fabs(clearance_angle) > fabs(passing_clearance_max)) {
    clearance_angle = passing_clearance_max;
  }

  // Calculate the deflection angle (angle between the line connecting
  // the ball and the support attacker and the heading of the support
  // attacker towards the goal). Deflection angles more and less than a
  // threshold will be penalized to prevent the support attacker from
  // positioning in front of or behind the attacker

  // Consider the deflection angle from the current ball position.
  const Vector2f ball_to_robot = position - ball_pos;
  const Vector2f robot_to_goal = Heading(proposed_pose.angle);
  float deflection_angle = acos(robot_to_goal.dot(-ball_to_robot) /
                                (robot_to_goal.norm() * ball_to_robot.norm()));
  deflection_angle = AngleMod(deflection_angle);
  //   if (RadToDeg(deflection_angle) > 180.0) {
  //       deflection_angle = DegToRad(360.0) - deflection_angle;
  //   }
  deflection_angle = fabs(deflection_angle);
  float deflection_normalized = pow(DegToRad(45.0) - deflection_angle, 2);
  deflection_normalized = deflection_normalized / M_PI;

  // Cost calculation (Option#1): Multiplication of the pass and goal angles
  goal_angle_width = goal_angle_width + goal_open_angle_bias;
  //   float goal_angle_width_normalized = goal_angle_width / M_PI;
  //   float clearance_angle_normalized = clearance_angle / M_PI;

  //   float ang_multiplication =
  //       goal_angle_width_normalized *
  //       clearance_angle_normalized;
  //   cost = 1.0 / (1.0 + ang_multiplication);

  //   cost = (cost + deflection_normalized) / 2;

  //   Cost calculation (Option#2): linear combination of the pass and goal
  //   angles
  float cost_goal_view = 1.0 / (1.0 + goal_angle_width);
  float aggressive_cost_goal_view = 1.0 / (1.0 + aggressive_goal_angle_width);
  float cost_attacker_view = 1.0 / (1.0 + clearance_angle);
  const float kFieldInset = 1000;
  float cost_x = position.x() - (kFieldXMax - kFieldInset);
  cost_x = fabs(cost_x) / (kFieldXMax - kFieldInset);
  const float kBallXThreshold = 2000;
  *aggressive_cost = 5.0;
  float cost2 = 1.0;
  if (ball_pos.x() <= kBallXThreshold) {
    cost = .3 * cost_goal_view + (.4) * cost_attacker_view + .3 * cost_x;
    cost2 = .5 * aggressive_cost_goal_view + (.5) * cost_attacker_view +
            (.1) * deflection_normalized + 0.1 * cost_x;
  } else {
    cost = .5 * cost_goal_view + (.5) * cost_attacker_view +
           (.5) * deflection_normalized + .01 * cost_x;
    cost2 = .5 * aggressive_cost_goal_view + (.5) * cost_attacker_view +
            (.5) * deflection_normalized + .5 * cost_x;
  }

  if (cost > 1.0) {
    cost = 1.0;
  }
  if (cost2 > 1.0) {
    cost2 = 1.0;
  }
  *aggressive_cost = cost2;

  return cost;
}

void WritePassFunctionData(
    const WorldState& world_state, SoccerState* soccer_state,
    float (*score_func)(Vector2f ball_pose,
                        vector<unsigned int> robots_to_ignore,
                        Vector2f pos_to_eval, const WorldState& world_state)) {
  vector<unsigned int> ignore_these;
  // Find the attacker_index
  size_t our_robots_num = world_state.GetOurRobots().GetElementCount();
  int attacker_index = 0;
  for (size_t k = 0; k < our_robots_num; k++) {
    if (soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::PRIMARY_ATTACKER ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::DIRECT_FREE_KICKER ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::KICKOFF ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::PENALTY_KICK ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::DIRECT_FREE_KICKER ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::INDIRECT_FREE_KICKER) {
      attacker_index = k;
    }
  }
  ignore_these.push_back(attacker_index);
  for (size_t k = 0; k < our_robots_num; k++) {
    if (soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
        tactics::SETUP_ATTACKER) {
      ignore_these.push_back(k);
    }
  }
  std::ofstream file;
  file.open("pass_scores.csv");
  file << "X,Y,Score\n";
  const Vector2f ball_pose = world_state.GetBallPosition().position;
  const float search_offset_left = 2 * kFieldXMax;
  const float search_offset_right = 2 * kFieldXMax;
  const float step_size = kRobotRadius;
  float left_edge = -kFieldXMax;
  if (ball_pose.x() > search_offset_left) {
    left_edge = ball_pose.x() - search_offset_left;
  }
  for (float x = left_edge;
       (x < kFieldXMax && x < ball_pose.x() + search_offset_right);
       x += step_size) {
    for (float y = -kFieldYMax; y < kFieldYMax; y += step_size) {
      const Vector2f position = {x, y};
      const float score =
          score_func(ball_pose, ignore_these, position, world_state);
      file << x << "," << y << "," << score << "\n";
    }
  }
}

struct Less {
  bool operator()(PassScore a, PassScore b) { return a.score_ < b.score_; }
};

void TargetEvaluator::ResetPassArray() {
  for (size_t i = 0; i < pass_array_.size(); ++i) {
    pass_array_[i].position_ = {20, 20};
    pass_array_[i].score_ = 10;
    aggressive_pass_array_[i].position_ = {20, 20};
    aggressive_pass_array_[i].score_ = 10;
  }
}

void TargetEvaluator::BuildPassingArray(
    const WorldState& world_state, SoccerState* soccer_state,
    float (*score_func)(Vector2f ball_pose,
                        vector<unsigned int> robots_to_ignore,
                        Vector2f pos_to_eval, const WorldState& world_state,
                        float* aggressive_score)) {
  float left_edge = -kFieldXMax;
  vector<unsigned int> ignore_these;
  // Find the primary and support attackers so we can ignore them
  // when calculating obstacles
  size_t our_robots_num = world_state.GetOurRobots().GetElementCount();
  int attacker_index = kNoPassRobot;
  for (size_t k = 0; k < our_robots_num; k++) {
    if (soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::PRIMARY_ATTACKER ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::BALL_PLACEMENT ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::DIRECT_FREE_KICKER ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::KICKOFF ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::PENALTY_KICK ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::DIRECT_FREE_KICKER ||
        soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
            tactics::INDIRECT_FREE_KICKER) {
      attacker_index = k;
    }
  }
  Vector2f ball_pose = world_state.GetBallPosition().position;
  if (attacker_index != kNoPassRobot) {
    ball_pose = soccer_state->GetRobotByOurRobotIndex(attacker_index)
                    .GetGoal()
                    .translation;
  }
  ignore_these.push_back(attacker_index);
  for (size_t k = 0; k < our_robots_num; k++) {
    if (soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
        tactics::SETUP_ATTACKER) {
      ignore_these.push_back(k);
    }
  }
  // Clip the search space to the left side of the field.
  if (ball_pose.x() - kSearchOffsetLeft >= -kFieldXMax) {
    left_edge = ball_pose.x() - kSearchOffsetLeft;
  }

  const float kFieldInset = 4 * kRobotRadius;
  float bottom_edge = -kFieldYMax + kFieldInset;
  if (ball_pose.y() - kSearchOffsetY >= bottom_edge) {
    bottom_edge = ball_pose.y() - kSearchOffsetY;
  }
  // Reset the pass array to high values
  ResetPassArray();
  // Iterate over an area around the ball and score each point.
  int i = -1;
  int count_x = 0;

  int count_y = 0;
  for (float x = left_edge;
       (x < kFieldXMax && x < ball_pose.x() + kSearchOffsetRight);
       x += kStepSize) {
    count_x++;
    count_y = 0;
    for (float y = bottom_edge;
         y < ball_pose.y() + kSearchOffsetY && y < kFieldYMax - kFieldInset;
         y += kStepSize) {
      i++;
      count_y++;
      const Vector2f position = {x, y};
      // If the position is too close to the ball then there's no reason
      // to evaluate it, as we want a buffer around the ball.
      if (geometry::EuclideanDistance(position, ball_pose) < kBallPadding) {
        continue;
      }
      // Calculate the score based on the passed score function.
      float aggressive_score = 1.0;
      float score = score_func(ball_pose, ignore_these, position, world_state,
                               &aggressive_score);
      // Determine if this location was previously a support goal
      // if it was record it's new value for hysterisis checks.
      // TODO(jaholtz) the comparison of vectors here is a bug,
      // floats are being compared with ==
      std::pair<Vector2f, bool> position_pair(position, true);
      int index = std::distance(setup_positions_.begin(),
                            std::find(setup_positions_.begin(),
                                      setup_positions_.begin()
                                      + setup_positions_.size(),
                                      position_pair));
      int aggressive_index = 50;
      aggressive_index =
          std::distance(aggressive_setup_positions_.begin(),
                        std::find(aggressive_setup_positions_.begin(),
                                  aggressive_setup_positions_.begin() +
                                      aggressive_setup_positions_.size(),
                                  position));
      if (index < static_cast<int>(setup_positions_.size())) {
        old_pose_scores_[index] = score;
      }
      if (aggressive_index <
          static_cast<int>(aggressive_setup_positions_.size())) {
        aggressive_old_pose_scores_[index] = aggressive_score;
      }
      // Set the new score
      pass_array_.at(i).position_ = position;
      pass_array_.at(i).score_ = score;
      aggressive_pass_array_.at(i).position_ = position;
      aggressive_pass_array_.at(i).score_ = aggressive_score;
    }
  }
  // Sort the scores (scores are currently costs, so sorted in ascending order)
  // This needs to be descending for actual scores.
  std::sort(pass_array_.begin(), pass_array_.begin() + pass_array_.size(),
            Less());
  std::sort(aggressive_pass_array_.begin(),
            aggressive_pass_array_.begin() + aggressive_pass_array_.size(),
            Less());
  const Vector2f best_position = pass_array_.at(0).position_;
  const Vector2f best_kick_vector = best_position - ball_pose;
  soccer_state->best_pass_angle_ = Angle(best_kick_vector);
  soccer_state->best_pass_position_ = best_position;
}

bool TargetEvaluator::TooClose(const Vector2f& pose_1, const Vector2f& pose_2) {
  const float distance = geometry::EuclideanDistance(pose_1, pose_2);
  return distance < kSupportAttackerPadding;
}

bool TargetEvaluator::Blocking(const Vector2f& pose_1, const Vector2f& pose_2,
                               const Vector2f& ball_position) {
  const RobotObstacle robot({0, pose_2});
  return robot.LineCollision(ball_position, pose_1, kRobotRadius * 5);
}

// Does this path cross between the ball and the attacker.
bool Interference(const Vector2f& robot_pose, const Vector2f& target_pose,
                  const Vector2f& attacker_pose, const Vector2f& ball_pose) {
  float intersection_distance;
  Vector2f intersect_point;
  const float distance = EuclideanDistance(attacker_pose, ball_pose);
  if (distance < kRobotRadius * 3) {
    return false;
  }
  return RayIntersect(robot_pose, target_pose, attacker_pose, ball_pose,
                      &intersection_distance, &intersect_point);
}

TargetEvaluator::TargetEvaluator() {
  ResetPassArray();
  setup_positions_.fill({{0, 0}, false});
  old_pose_scores_.fill(99999);
  aggressive_setup_positions_.fill({0, 0});
  aggressive_old_pose_scores_.fill(99999);
}

void TargetEvaluator::AssignSupportPositions(const WorldState& world_state,
                                             SoccerState* soccer_state) {
  vector<Vector2f> assigned_poses;
  vector<OurRobotIndex> receivers;
  vector<OurRobotIndex> aggressive_receivers;
  std::array<float, 8> setup_scores;
  setup_scores.fill(5.0);
  std::array<float, 8> aggressive_setup_scores;
  setup_scores.fill(5.0);
  const Vector2f& ball_pose = world_state.GetBallPosition().position;
  // Find the support attackers (receivers)
  size_t our_robots_num = world_state.GetOurRobots().GetElementCount();
  OurRobotIndex prime_attacker = kNoPassRobot;
  for (size_t k = 0; k < our_robots_num; k++) {
    if (soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
        tactics::SETUP_ATTACKER) {
      receivers.push_back(k);
    }
    // Coercive attackers
    if (soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
        tactics::COERCIVE_ATTACKER) {
      aggressive_receivers.push_back(k);
    }
    if (soccer_state->GetRobotByOurRobotIndex(k).current_tactic_ ==
        tactics::PRIMARY_ATTACKER) {
      prime_attacker = k;
    }
  }
  // Assign a position to each receiver
  for (PassScore score : pass_array_) {
    // If no receivers no reason to continue
    if (receivers.size() > 0) {
      bool invalid = false;
      // Check if the position is invalidated by other poses.
      for (Vector2f pose : assigned_poses) {
        // Must keep distance from other robots
        if (TooClose(pose, score.position_)) {
          invalid = true;
        }
        // Must not block angles to other robots.
        if (Blocking(pose, score.position_, ball_pose)) {
          invalid = true;
        }
      }
      // Don't consider invalidated poses
      if (invalid) {
        continue;
      }
      // Find the receiver closest to this position.
      OurRobotIndex best_receiver = 42;
      size_t best_index = receivers.size();
      float best_distance = std::numeric_limits<float>::max();
      for (size_t i = 0; i < receivers.size(); ++i) {
        pose_2d::Pose2Df position =
            world_state.GetOurRobotPosition(receivers[i]).position;
        if (prime_attacker != kNoPassRobot) {
          pose_2d::Pose2Df attacker_pose =
              world_state.GetOurRobotPosition(prime_attacker).position;
          if (Interference(position.translation, score.position_,
                           attacker_pose.translation, ball_pose)) {
            continue;
          }
        }
        float intersect_distance;
        Vector2f intersect_point;
        if (RayIntersect(ball_pose, kTheirGoalCenter, position.translation,
                         score.position_, &intersect_distance,
                         &intersect_point)) {
          continue;
        }
        float distance =
            geometry::SquaredDistance(position.translation, score.position_);
        if (distance < best_distance) {
          best_receiver = receivers[i];
          best_distance = distance;
          best_index = i;
        }
      }
      // Assign the receiver it's new position
      if (best_index < receivers.size()) {
        // Only assign a new position if the score is sufficiently improved.
        if (fabs(old_pose_scores_[best_receiver] - score.score_)
            > kScoreHysterisis) {
          setup_positions_[best_receiver] = {score.position_, true};
          setup_scores[best_receiver] = score.score_;
        } else {
          setup_scores[best_receiver] = old_pose_scores_[best_receiver];
        }
        assigned_poses.push_back(setup_positions_[best_receiver].first);
        receivers.erase(receivers.begin() + best_index);
        // Set the goal position in the tactic
        soccer_state->
            GetRobotByOurRobotIndex(best_receiver).SetGoal({0,
                                    setup_positions_[best_receiver].first});
      }
    }
  }
  // ---------------------------------------------------------------------
  // Assign a position to each coercive attacker
  for (PassScore score : aggressive_pass_array_) {
    //     LOG(ERROR) << "POTENTIAL SCORE: " << score.score_;
    // If no receivers no reason to continue
    if (aggressive_receivers.size() > 0) {
      bool invalid = false;
      // Check if the position is invalidated by other poses.
      for (Vector2f pose : assigned_poses) {
        // Must keep distance from other robots
        if (TooClose(pose, score.position_)) {
          invalid = true;
        }
        // Must not block angles to other robots.
        if (Blocking(pose, score.position_, ball_pose)) {
          invalid = true;
        }
      }
      //       Don't consider invalidated poses
      if (invalid) {
        continue;
      }
      // Find the receiver closest to this position.
      OurRobotIndex best_receiver = 42;
      size_t best_index = aggressive_receivers.size();
      float best_distance = std::numeric_limits<float>::max();
      for (size_t i = 0; i < aggressive_receivers.size(); ++i) {
        pose_2d::Pose2Df position =
            world_state.GetOurRobotPosition(aggressive_receivers[i]).position;
        if (prime_attacker != kNoPassRobot) {
          pose_2d::Pose2Df attacker_pose =
              world_state.GetOurRobotPosition(prime_attacker).position;
          if (Interference(position.translation, score.position_,
                           attacker_pose.translation, ball_pose)) {
            continue;
          }
        }
        float intersect_distance;
        Vector2f intersect_point;
        if (ball_pose.x() > 1000) {
          if (RayIntersect(ball_pose, kTheirGoalCenter, position.translation,
                           score.position_, &intersect_distance,
                           &intersect_point)) {
            continue;
          }
        }
        float distance =
            geometry::SquaredDistance(position.translation, score.position_);
        if (distance < best_distance) {
          best_receiver = aggressive_receivers[i];
          best_distance = distance;
          best_index = i;
        }
      }
      // Assign the receiver it's new position
      if (best_index < aggressive_receivers.size()) {
        // Only assign a new position if the score is sufficiently improved.
        if (fabs(aggressive_old_pose_scores_[best_receiver] - score.score_) >
            kScoreHysterisis) {
          aggressive_setup_positions_[best_receiver] = score.position_;
          aggressive_setup_scores[best_receiver] = score.score_;
        } else {
          aggressive_setup_scores[best_receiver] =
              aggressive_old_pose_scores_[best_receiver];
        }
        assigned_poses.push_back(aggressive_setup_positions_[best_receiver]);
        aggressive_receivers.erase(aggressive_receivers.begin() + best_index);
        // Set the goal position in the tactic
        soccer_state->GetRobotByOurRobotIndex(best_receiver)
            .SetGoal({0, aggressive_setup_positions_[best_receiver]});
      }
    }
  }
  soccer_state->setup_positions_ = setup_positions_;
  old_pose_scores_ = soccer_state->setup_scores_;
  soccer_state->setup_scores_ = setup_scores;
  aggressive_old_pose_scores_ = soccer_state->aggressive_setup_scores_;
  soccer_state->aggressive_setup_positions_ = aggressive_setup_positions_;
  soccer_state->aggressive_setup_scores_ = aggressive_setup_scores;
}

void TargetEvaluator::Update(
    const WorldState& world_state, SoccerState* soccer_state,
    float (*score_func)(Vector2f ball_pose,
                        vector<unsigned int> robots_to_ignore,
                        Vector2f pos_to_eval, const WorldState& world_state,
                        float* aggressive_score)) {
  BuildPassingArray(world_state, soccer_state, score_func);
  AssignSupportPositions(world_state, soccer_state);
}

Vector2f TargetEvaluator::GetSetupPosition(const OurRobotIndex& robot) {
  return setup_positions_[robot].first;
}

}  // namespace offense
