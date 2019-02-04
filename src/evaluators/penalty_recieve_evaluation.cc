// Copyright 2017 - 2018 slane@cs.umass.edu, jaholtz@cs.umass.edu
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

#include "evaluators/penalty_recieve_evaluation.h"

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

PenaltyRecieveEvaluator::PenaltyRecieveEvaluator() {}

void PenaltyRecieveEvaluator::SetRecieveTargets(
    const WorldState& world_state, const SoccerState& soccer_state) {
  vector<OurRobotIndex> recieving_indices;
  Vector2f ball_position = world_state.GetBallPosition().position;

  for (const SoccerRobot& robot : soccer_state.GetAllSoccerRobots()) {
    if (robot.enabled_ && robot.current_tactic_ == tactics::PENALTY_RECIEVE) {
      recieving_indices.push_back(robot.our_robot_index_);
    }
  }

  vector<Pose2Df> poses_to_assign;
  // Assigning at most four robots to follow, if their are more, they will be
  // assigned their current position as the goal position.
  for (size_t index = 0; index < recieving_indices.size(); index++) {
    if (soccer_state.IsOurPenaltyKick()) {
      if (index == 0) {
        Pose2Df desired_pose(
            0, ball_position.x() - 2500 - kRobotRadius - 30 - kEpsilon, 0);
        poses_to_assign.push_back(desired_pose);
      } else if (index % 2 == 0) {
        Pose2Df desired_pose(
            0, ball_position.x() - 500 - kRobotRadius - 30 - kEpsilon,
            -200 * index);
        poses_to_assign.push_back(desired_pose);
      } else {
        Pose2Df desired_pose(
            0, ball_position.x() - 500 - kRobotRadius - 30 - kEpsilon,
            200 * index);
        poses_to_assign.push_back(desired_pose);
      }
    } else if (soccer_state.IsTheirPenaltyKick()) {
      if (index == 0) {
        Pose2Df desired_pose(
            0, ball_position.x() + 500 + kRobotRadius + 30 + kEpsilon, 750);
        poses_to_assign.push_back(desired_pose);
      } else if (index == 1) {
        Pose2Df desired_pose(
            0, ball_position.x() + 500 + kRobotRadius + 30 + kEpsilon, -750);
        poses_to_assign.push_back(desired_pose);
      } else if (index == 2) {
        Pose2Df desired_pose(0, ball_position.x() + 1250, 0);
        poses_to_assign.push_back(desired_pose);
      } else if (index == 3) {
        Pose2Df desired_pose(0, ball_position.x() + 1400, -1750);
        poses_to_assign.push_back(desired_pose);
      } else if (index == 4) {
        Pose2Df desired_pose(0, ball_position.x() + 1400, 1750);
        poses_to_assign.push_back(desired_pose);
      } else if (index == 5) {
        Pose2Df desired_pose(0, ball_position.x() + 2000, 200);
        poses_to_assign.push_back(desired_pose);
      } else if (index == 6) {
        Pose2Df desired_pose(0, ball_position.x() + 2000, -200);
        poses_to_assign.push_back(desired_pose);
      } else {
        poses_to_assign.push_back(Pose2Df(M_PI, 0, 0));
      }
    }
  }

  if (poses_to_assign.size() > 0) {
    AssignPoses(recieving_indices, world_state, soccer_state, poses_to_assign);
  }
}

void PenaltyRecieveEvaluator::AssignPoses(
    const vector<OurRobotIndex>& robot_indices, const WorldState& world_state,
    const SoccerState& soccer_state, const vector<Pose2Df>& poses_to_assign) {
  vector<bool> assigned_pose(robot_indices.size(), false);

  for (size_t i = 0; i < poses_to_assign.size(); ++i) {
    float closest_squared_distance = 10e20;
    int closest_robot_index = -1;

    Pose2Df robot_pose = world_state.GetOurRobotPosition(i).position;
    Pose2Df target_pose = poses_to_assign[i];
    for (size_t j = 0; j < robot_indices.size(); ++j) {
      if (!assigned_pose[j]) {
        float current_squared_distance =
            SquaredDistance(robot_pose.translation, target_pose.translation);

        if (current_squared_distance < closest_squared_distance) {
          closest_squared_distance = current_squared_distance;
          closest_robot_index = j;
        }
      }
    }

    const SoccerRobot& robot = soccer_state.GetRobotByOurRobotIndex(
        robot_indices[closest_robot_index]);
    robot.SetGoal(target_pose);
    assigned_pose[i] = true;
  }
}

}  // namespace defense
