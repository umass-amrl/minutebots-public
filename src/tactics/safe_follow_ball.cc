// Copyright 2017 - 2018 slane@cs.umass.edu, , jaholtz@cs.umass.edu
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

#include "tactics/safe_follow_ball.h"

#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using state::WorldState;
using state::WorldRobot;
using state::SharedState;
using state::SharedRobotState;
using std::cos;
using std::sin;
using std::endl;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using geometry::RayIntersect;

namespace tactics {
SafeFollowBall::SafeFollowBall(const WorldState& world_state,
                             TacticArray* tactic_list,
                             SharedState* shared_state,
                             OurRobotIndex our_robot_index,
                             state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

void SafeFollowBall::Init() {
  robot_goal_pose_.Set(0.0f, Vector2f(1000, 1000));
}

// Cost is currently the distance to the ball.
float SafeFollowBall::GetCost() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f current_trans = current_pose.translation;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  float distance = sqrt(pow(current_trans.x() - ball_pose.x(), 2) +
                        pow(current_trans.y() - ball_pose.y(), 2));
  return distance;
}

Vector2f SafeFollowBall::ProjectToSafety(const Vector2f& start_point,
                                         const Vector2f& end_point) {
  zone::FieldZone field_zone(zone::FULL_FIELD);

  Vector2f new_end = end_point;
  if (!field_zone.IsInZone(end_point, kRobotRadius + kDefaultSafetyMargin)) {
    Vector2f v1(field_zone.min_x_, field_zone.min_y_);
    Vector2f v2(field_zone.min_x_, field_zone.max_y_);
    Vector2f v3(field_zone.max_x_, field_zone.min_y_);
    Vector2f v4(field_zone.max_x_, field_zone.max_y_);

    float square_dist;
    float min_square_dist = INFINITY;
    Vector2f intersect;
    Vector2f min_intersect;
    bool does_intersect;

    Vector2f displace = new_end - start_point;

    does_intersect = RayIntersect(start_point, displace, v1, v2,
                                  &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    does_intersect = RayIntersect(start_point, displace, v1, v3,
                                  &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    does_intersect = RayIntersect(start_point, displace, v4, v2,
                                  &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    does_intersect = RayIntersect(start_point, displace, v4, v3,
                                  &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    new_end = min_intersect
    - (kRobotRadius + kDefaultSafetyMargin) * displace.normalized();
  }
  return new_end;
}

void SafeFollowBall::Run() {
  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  auto pose = world_state_.GetOurRobotPosition(our_robot_index_);
  robot_goal_pose_.translation = ProjectToSafety(pose.position.translation,
                                                 robot_goal_pose_.translation);
  controller->SetGoal(robot_goal_pose_);
  controller->Run();
}

void SafeFollowBall::Reset() {
  robot_goal_pose_.Set(0.0f, Vector2f(1000, 1000));
}

void SafeFollowBall::SetGoal(const Pose2Df& pose) {
  robot_goal_pose_ = pose;
}

bool SafeFollowBall::IsComplete() { return true; }

}  // namespace tactics
