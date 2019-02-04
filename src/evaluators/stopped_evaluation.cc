// Copyright 2017 slane@cs.umass.edu, jaholtz@cs.umass.edu
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

#include "evaluators/stopped_evaluation.h"

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/soccer_state.h"
#include "state/world_ball.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic_index.h"

STANDARD_USINGS;
using Eigen::Vector2f;
using geometry::Angle;
using geometry::RayIntersect;
using geometry::FurthestFreePointCircle;
using math_util::Sign;
using math_util::Sq;
using geometry::EuclideanDistance;
using geometry::SquaredDistance;
using geometry::SafeVectorNorm;
using offense::AimOption;
using offense::CalculateAimOptions;
using pose_2d::Pose2Df;
using state::WorldState;
using state::WorldRobot;
using state::SoccerRobot;
using state::SoccerState;
using std::cos;
using std::sin;
using std::pair;
using tactics::TacticIndex;
using std::vector;

namespace defense {

StoppedEvaluator::StoppedEvaluator() {}

void StoppedEvaluator::SetStoppedTargets(const WorldState& world_state,
                                         const SoccerState& soccer_state) {
  vector<OurRobotIndex> following_indices;

  for (const SoccerRobot& robot : soccer_state.GetAllSoccerRobots()) {
    if (robot.enabled_ && robot.current_tactic_ == tactics::SAFE_BALL_FOLLOW) {
      following_indices.push_back(robot.our_robot_index_);
    }
  }

  const Vector2f& ball_position = world_state.GetBallPosition().position;
  vector<Pose2Df> poses_to_assign;
  int count = 0;
  // Assigning at most four robots to follow, if their are more, they will be
  // assigned their current position as the goal position.
  const float kRobotDistance = kStoppedRadius;
  for (OurRobotIndex index : following_indices) {
    auto robot = world_state.GetOurRobotPosition(index);
    // Get behind the ball along the line to their goal.

//     if (count == 0) {
//       bool pushed = false;
//       for (auto opponent : world_state.GetTheirRobots()) {
//       Vector2f dist_to_ball = ball_position - opponent.position.translation;
//         const float distance = dist_to_ball.norm();
//         if (fabs(distance) < 400 && !pushed) {
//           Vector2f goal_trans = kRobotDistance * dist_to_ball.normalized() +
//               ball_position;
//           Vector2f neg_ray = -dist_to_ball;
//           float angle = Angle(neg_ray);
//           Pose2Df target_pose(angle, goal_trans);
//           poses_to_assign.push_back(target_pose);
//           pushed = true;
//         }
//       }
//       if (!pushed) {
//         Vector2f rayToTheirGoal = (kTheirGoalCenter - ball_position);
//         Vector2f goal_trans = kRobotDistance * rayToTheirGoal.normalized() +
//                               ball_position;
//         Vector2f neg_ray = -rayToTheirGoal;
//         float angle = Angle(neg_ray);
//         Pose2Df target_pose(angle, goal_trans);
//         poses_to_assign.push_back(target_pose);
//       }
//     // Block the line to our goal.
//     } else
  if (count == 0) {
      Vector2f rayToOurGoal = kOurGoalCenter - ball_position;
      Vector2f goal_trans = kRobotDistance * rayToOurGoal.normalized() +
                            ball_position;
      Vector2f neg_ray = -rayToOurGoal;
      float angle = Angle(neg_ray);
      Pose2Df target_pose(angle, goal_trans);
      poses_to_assign.push_back(target_pose);
    // Get behind the ball along the line to our Left goalpost
    } else if (count == 1) {
      Vector2f rayToGoalpost = (kOurGoalL - ball_position);
      if (rayToGoalpost.norm() == 0) {
        poses_to_assign.push_back(robot.position);
      } else {
        Vector2f goal_trans = kRobotDistance * rayToGoalpost.normalized() +
                              ball_position;
        Vector2f neg_ray = -rayToGoalpost;
        float angle = Angle(neg_ray);
        Pose2Df target_pose(angle, goal_trans);
        poses_to_assign.push_back(target_pose);
      }
    // Get behind the ball along the line to our goal.
    } else if (count == 2) {
      Vector2f rayToOurGoal = -(kOurGoalCenter - ball_position);
      Vector2f goal_trans = kRobotDistance * rayToOurGoal.normalized() +
                            ball_position;
      Vector2f neg_ray = -rayToOurGoal;
      float angle = Angle(neg_ray);
      Pose2Df target_pose(angle, goal_trans);
      poses_to_assign.push_back(target_pose);
    } else {
      poses_to_assign.push_back(robot.position);
    }
    count++;
  }

  if (poses_to_assign.size() > 0) {
    AssignPoses(following_indices, world_state, soccer_state, &poses_to_assign);
  }
}

void StoppedEvaluator::AssignPoses(
  const vector<OurRobotIndex>& robot_indices,
  const WorldState& world_state, const SoccerState& soccer_state,
  vector<Pose2Df>* poses_to_assign) {
  const Vector2f& ball_position = world_state.GetBallPosition().position;

  // Resolve collisions between assigned poses
  for (unsigned int i = 1; i < poses_to_assign->size(); i++) {
    Pose2Df pose = (*poses_to_assign)[i];
    int movement_direction = 0;
    for (unsigned int j = 0; j < i; j++) {
      const Pose2Df& reference_pose = (*poses_to_assign)[j];

      if (SquaredDistance(pose.translation, reference_pose.translation) <
          kEpsilon) {
        pose.translation.y() += 0.01;
      }

      Vector2f direction = (pose.translation - reference_pose.translation);
      float distance = SafeVectorNorm(direction);
      if (distance < kStoppedDefenderMargin * 1.5) {
        Vector2f ball_to_pose = pose.translation - ball_position;
        float current_angle = Angle(ball_to_pose);
        Vector2f ball_to_reference = reference_pose.translation - ball_position;
        float reference_angle = Angle(ball_to_reference);

        if (movement_direction == 0) {
          float current_angle_diff = AngleDiff(current_angle, reference_angle);
          movement_direction = Sign(current_angle_diff);
        }

        float new_angle = reference_angle +
                          movement_direction * kStoppedMinimumAngle;

        pose.translation.x() = kStoppedRadius*cos(new_angle) +
                               ball_position.x();
        pose.translation.y() = kStoppedRadius*sin(new_angle) +
                               ball_position.y();
      }
    }
    (*poses_to_assign)[i] = pose;
  }

  for (size_t i = 0; i < poses_to_assign->size(); ++i) {
    Pose2Df target_pose = (*poses_to_assign)[i];
    const SoccerRobot& robot =
        soccer_state.GetRobotByOurRobotIndex(robot_indices[i]);
    robot.SetGoal(target_pose);
  }
}

}  // namespace defense
