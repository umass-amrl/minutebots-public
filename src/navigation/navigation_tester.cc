// Copyright 2018 slane@cs.umass.edu
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

#include <signal.h>

#include <array>
#include <atomic>
#include <vector>

#include "logging/logger.h"
#include "navigation/navigation_util.h"
#include "navigation/stox_planner.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "state/position_velocity_state.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "util/timer.h"

using logger::NetLogger;
using navigation::SmoothPath;
using navigation::StoxPlanner;
using obstacle::ObstacleFlag;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
using state::PositionVelocityState;
using state::SoccerRobot;
using state::SoccerState;
using state::WorldState;
using std::array;
using std::atomic_bool;
using std::vector;

// Used for handling the lifetime and eventual shutdown of the main RoboCup
// stack.
static atomic_bool shutdown_flag(false);

void SigHandler(int signo) {
  if (signo == SIGINT) {
    shutdown_flag = true;
  }
}

int main(int argc, char** argv) {
  // Construct the PositionVelocityState
  PositionVelocityState pvs;

  array<Vector2f, kNumTeams * kMaxTeamRobots> goals;

  for (unsigned int i = 0; i < kNumTeams * kMaxTeamRobots; i++) {
    float angle = i * 2 * M_PI / (kNumTeams * kMaxTeamRobots);
    float inverse = AngleMod(angle + M_PI);
    float radius = 100.0f;
    float goal_radius = 1000.0f;

    PositionVelocityState::RobotPositionVelocity world_robot(
        i, pose_2d::Pose2Df(inverse, radius * cos(angle), radius * sin(angle)),
        pose_2d::Pose2Df(0.0, cos(inverse), sin(inverse)),
        pose_2d::Pose2Df(inverse, radius * cos(angle), radius * sin(angle)),
        {0, 0, 0}, GetMonotonicTime(), 1.0f);
    if (i % 2 == 0) {
      pvs.GetMutableOurTeamRobots()->InsertBack(world_robot);
    } else {
      pvs.GetMutableTheirTeamRobots()->InsertBack(world_robot);
    }
    goals[i] = Vector2f(goal_radius * cos(inverse), goal_radius * sin(inverse));
  }

  // Construct the WorldState
  WorldState world_state(&pvs, team::Team::BLUE);

  // Construct the Soccer State and update it
  state::SoccerState soccer_state(world_state, team::Team::BLUE);
  soccer_state.UpdateExistances();

  // Make a logger
  NetLogger logger(NetLogger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT));

  // Make a safety margin
  SafetyMargin margin;
  margin.SetMargin(ObstacleType::ROBOT, kNavigationRobotMargin);
  margin.SetMargin(ObstacleType::BALL, kNavigationBallMargin);

  // Make a planner object
  StoxPlanner planner;
  ObstacleFlag obstacles =
      ObstacleFlag::GetAllExceptTeam(world_state, soccer_state, 0, 0);
  planner.Init(obstacles);

  // Start the Loop
  RateLoop loop(kTransmitFrequency);
  while (!shutdown_flag) {
    for (const SoccerRobot& robot : soccer_state.GetAllSoccerRobots()) {
      if (robot.enabled_) {
        // Push Log with robot ID
        logger.LogPrintPush("Robot %X", robot.ssl_vision_id_);

        // Get current obstacles
        obstacles = ObstacleFlag::GetAllExceptTeam(world_state, soccer_state,
                                                   robot.our_robot_index_,
                                                   robot.our_robot_index_);

        // Get current position
        Vector2f current =
            world_state.GetOurRobotPosition(robot.our_robot_index_)
                .position.translation;

        // Get current goal
        Vector2f goal = goals[robot.our_robot_index_ * kNumTeams];

        // Update Planning object
        planner.Update(obstacles, margin, current, goal, &logger);

        // Plan
        const vector<Vector2f> plan = planner.Plan(&logger).second;

        // Smooth plan
        vector<Vector2f> smoothed;
        SmoothPath(obstacles, margin, true, &smoothed, &logger);

        // Draw plan
        Vector2f prev_point = current;
        for (const Vector2f& point : plan) {
          logger.AddLine(prev_point, point, 1, 0, 1, 0.75);
          prev_point = point;
        }

        // Pop Log
        logger.Pop();
      }
    }

    logger.SetWorldState(world_state);
    logger.SendData();
    logger.Clear();
    loop.Sleep();
  }
}
