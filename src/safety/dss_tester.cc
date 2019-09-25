// Copyright 2016 - 2019 srabiee@cs.umass.edu
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

#include <glog/logging.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <bitset>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

#include "constants/constants.h"
#include "logging/logger.h"
#include "net/netraw.h"
#include "netraw_test_message.pb.h"
#include "plays/skills_tactics_plays.h"
#include "safety/dss.h"
#include "soccer/executor.h"
#include "soccer/kalmanupdate.h"
#include "state/direction.h"
#include "state/position_velocity_state.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/team.h"
#include "state/world_state.h"
#include "tactics/tactic.h"
#include "third_party/optionparser-1.4/optionparser.h"
#include "util/colorize.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_queue.h"
#include "util/timer.h"

using std::cout;
using std::endl;
using std::vector;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector3f;
using logger::Logger;
using state::SoccerState;
using state::SoccerRobot;
using state::SharedRobotState;
using state::WorldState;
using state::WorldRobot;
using state::PositionVelocityState;
using safety::DSS;

int InitializeRobots(state::PositionVelocityState* pvs) {
  // Number of robots initialized for each team
  int robot_num = 6;
  float radius = 1000.0;
  float velocity = 4000.0;

  // The theta for each robot in polar coordinate
  vector<float> pos_theta(robot_num);
  vector<float> pos_x(robot_num);
  vector<float> pos_y(robot_num);
  vector<float> pos_theta_their_team(robot_num);
  vector<float> pos_x_their_team(robot_num);
  vector<float> pos_y_their_team(robot_num);

  // The direction of each robot as part of their pose
  vector<float> rob_direction(robot_num);
  vector<float> vel_x(robot_num);
  vector<float> vel_y(robot_num);
  vector<float> rob_direction_their_team(robot_num);
  vector<float> vel_x_their_team(robot_num);
  vector<float> vel_y_their_team(robot_num);

  // Calculate the desired pose and velocity for both teams' robots
  for (int i = 0; i < robot_num; ++i) {
    pos_theta[i] = -M_PI / 2 + (i + 1) * M_PI / (robot_num + 1);
    pos_x[i] = radius * cos(pos_theta[i]);
    pos_y[i] = radius * sin(pos_theta[i]);
    rob_direction[i] = pos_theta[i] - M_PI;
    vel_x[i] = velocity * cos(rob_direction[i]);
    vel_y[i] = velocity * sin(rob_direction[i]);

    pos_theta_their_team[i] = M_PI / 2 + (i + 1) * M_PI / (robot_num + 1);
    pos_x_their_team[i] = radius * cos(pos_theta_their_team[i]);
    pos_y_their_team[i] = radius * sin(pos_theta_their_team[i]);
    rob_direction_their_team[i] = pos_theta_their_team[i] - M_PI;
    vel_x_their_team[i] = velocity * cos(rob_direction_their_team[i]);
    vel_y_their_team[i] = velocity * sin(rob_direction_their_team[i]);
  }

  // Initialize our robots
  for (int i = 0; i < robot_num; ++i) {
    Pose2Df robot_pose(rob_direction[i], pos_x[i], pos_y[i]);
    state::PositionVelocityState::RobotPositionVelocity robot(
        i, robot_pose, Pose2Df(0, vel_x[i], vel_y[i]), robot_pose,
        {0, 0, 0}, 0, 1);
    pvs->GetMutableOurTeamRobots()->InsertBack(robot);
  }

  // Initialize their robots
  for (int i = 0; i < robot_num; ++i) {
    Pose2Df robot_pose(rob_direction_their_team[i], pos_x_their_team[i],
                       pos_y_their_team[i]);
    state::PositionVelocityState::RobotPositionVelocity robot(
        i + robot_num, robot_pose,
        Pose2Df(0, vel_x_their_team[i], vel_y_their_team[i]), robot_pose,
        {0, 0, 0}, 0, 1);
    pvs->GetMutableTheirTeamRobots()->InsertBack(robot);
  }

  return 0;
}

void MergeRobotMessages(state::SoccerState* soccer_state,
                        const state::WorldState& world_state,
                        logger::NetLogger* logger) {
  vector<SoccerRobot>* mutable_soccer_robots =
      soccer_state->GetAllMutableSoccerRobots();
  for (SoccerRobot& robot : *mutable_soccer_robots) {
    if (robot.enabled_) {
      const auto& wr = world_state.GetOurRobotPosition(robot.our_robot_index_);
      const string robot_str = StringPrintf("Robot %X", wr.ssl_vision_id);
      logger->LogPrint(robot_str.c_str());
      logger->Push();
      logger->MergeLoggers(robot.bot_logger);
      logger->Pop();
      robot.bot_logger.Clear();
    }
  }
}

void SetCommandVelocities(state::SoccerState* soccer_state,
                          const state::WorldState& world_state) {
  vector<SoccerRobot>* mutable_soccer_robots =
      soccer_state->GetAllMutableSoccerRobots();
  for (SoccerRobot& robot : *mutable_soccer_robots) {
    if (robot.enabled_) {
      const auto& wr = world_state.GetOurRobotPosition(robot.our_robot_index_);
      Pose2Df current_rob_vel = wr.velocity;

      robot.shared_state->GetSharedState(robot.our_robot_index_)->velocity_x =
          current_rob_vel.translation(0);
      robot.shared_state->GetSharedState(robot.our_robot_index_)->velocity_y =
          current_rob_vel.translation(1);
    }
  }
}

void VisualizeVelocity(state::SoccerState* soccer_state,
                       const state::WorldState& world_state,
                       const Vector3f& color, logger::NetLogger* logger) {
  float circle_marker_radius = 20.0;
  float velocity_scalar = 10.0;

  vector<SoccerRobot>* mutable_soccer_robots =
      soccer_state->GetAllMutableSoccerRobots();
  for (SoccerRobot& robot : *mutable_soccer_robots) {
    if (robot.enabled_) {
      const auto& wr = world_state.GetOurRobotPosition(robot.our_robot_index_);
      Pose2Df current_rob_pos = wr.observed_pose;
      Pose2Df current_rob_vel = wr.velocity;

      Vector2f scaled_vel;
      scaled_vel(0) = robot.shared_state->GetSharedState(robot.our_robot_index_)
                          ->velocity_x;
      scaled_vel(1) = robot.shared_state->GetSharedState(robot.our_robot_index_)
                          ->velocity_y;
      scaled_vel = scaled_vel / velocity_scalar;

      logger->AddLine(Vector2f(current_rob_pos.translation(0),
                               current_rob_pos.translation(1)),
                      Vector2f(current_rob_pos.translation(0) + scaled_vel(0),
                               current_rob_pos.translation(1) + scaled_vel(1)),
                      color(0), color(1), color(2), 1);

      logger->AddCircle(
          Vector2f(current_rob_pos.translation(0) + scaled_vel(0),
                   current_rob_pos.translation(1) + scaled_vel(1)),
          circle_marker_radius, color(0), color(1), color(2), 1);
    }
  }
}

int main(int argc, char** argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 1;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk
  bool kDebug = true;

  // Initialize the world state
  state::PositionVelocityState pvs;
  InitializeRobots(&pvs);
  state::WorldState world_state(&pvs, team::Team::BLUE);
  state::SoccerState soccer_state(world_state);
  soccer_state.UpdateExistances();

  // Set the initialized robot velocities in the shared state
  SetCommandVelocities(&soccer_state, world_state);

  DSS dss(world_state, &soccer_state);
  RateLoop loop(kTransmitFrequency);
  logger::NetLogger logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
  logger.SetWorldState(world_state);

  Vector3f pre_dss_velocity_color(0, 0, 1);
  Vector3f post_dss_velocity_color(1, 0, 0);

  while (true) {
    // Reset the commanded velocities
    SetCommandVelocities(&soccer_state, world_state);

    if (kDebug) {
      // Visualize the robot velocities before calling DSS
      VisualizeVelocity(&soccer_state, world_state, pre_dss_velocity_color,
                        &logger);
    }

    double t_start = GetMonotonicTime();
    dss.MakeSafe({kMaxRobotAcceleration, kMaxRobotVelocity},
                 {kMaxRobotAcceleration, kMaxRobotVelocity},
                 1.0 / kTransmitFrequency);
    double t_end = GetMonotonicTime();

    if (kDebug) {
      LOG(WARNING) << "DSS time for 60 calls: " << 60 * (t_end - t_start);
      logger.SetWorldState(world_state);

      // Visualize the robot velocities after calling DSS
      VisualizeVelocity(&soccer_state, world_state, post_dss_velocity_color,
                        &logger);

      MergeRobotMessages(&soccer_state, world_state, &logger);

      logger.SendData();
      logger.Clear();
    }
    loop.Sleep();
  }
  return 0;
}
