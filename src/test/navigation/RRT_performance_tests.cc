// Copyright 2016 - 2018 kvedder@umass.edu
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
#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>

#include "constants/constants.h"
#include "graph/graph_util.h"
#include "math/poses_2d.h"
#include "navigation/RRT.h"
#include "navigation/navigation_util.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "performance.pb.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "test/navigation/scenario_generator.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "util/timer.h"

using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector3f;
using obstacle::Obstacle;
using obstacle::CircleObstacle;
using obstacle::ObstacleType;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using state::SoccerState;
using state::WorldState;
using state::WorldRobot;
using state::PositionVelocityState;

namespace navigation {
namespace rrt {

TEST(QualityTests, DumpRRTPath) {
  constexpr auto kNumSeeds = 100u;
  constexpr auto kNumPerformanceTrials = 50u;
  constexpr auto kMaxScaffoldBias = 0.8;
  constexpr auto kScaffoldBiasStep = 0.01;

  for (const auto& scenerio_type : navigation::test::GetScenerioTypes()) {
    MinuteBotsProto::Scenerio scenerio_proto;
    scenerio_proto.set_scenerio_name(
        navigation::test::GetScenerioName(scenerio_type));
    for (float scaffold_bias = 0; scaffold_bias < kMaxScaffoldBias;
         scaffold_bias += kScaffoldBiasStep) {
      LOG(WARNING) << "Scaffold bias: " << scaffold_bias;
      auto* configuration_proto = scenerio_proto.add_configuration_list();
      configuration_proto->mutable_rrt_config()->set_scaffold_bias(
          scaffold_bias);

      for (size_t scaffold_counter = 0; scaffold_counter < 2;
           ++scaffold_counter) {
        bool use_scaffold = (scaffold_counter == 0);
        for (size_t i = 0; i < kNumSeeds; ++i) {
          const uint64_t seed = i * 32797;
          MinuteBotsProto::Trial* trial_proto =
              configuration_proto->add_trial_list();
          trial_proto->set_seed(seed);
          trial_proto->set_scaffold(use_scaffold);
          for (size_t trial = 0; trial < kNumPerformanceTrials; ++trial) {
            auto scenerio = navigation::test::GetScenerio(scenerio_type);

            RRT rrt(scenerio.obstacle_flag, scenerio.margin, seed,
                    /* use_scaffolding = */ use_scaffold, scaffold_bias);

            auto start_time = GetMonotonicTime();

            rrt.Update(scenerio.obstacle_flag, scenerio.start, scenerio.goal,
                       scenerio.margin);
            auto plan = rrt.Plan();

            auto end_time = GetMonotonicTime();

            trial_proto->add_trial_times_list(end_time - start_time);

            if (trial == (kNumPerformanceTrials - 1)) {
              trial_proto->set_path_found(plan.first.first);
              trial_proto->set_samples(plan.first.second);
              trial_proto->set_path_distance(
                  (scenerio.goal - scenerio.start).norm());
              float path_length = 0;
              for (size_t j = 0; j < plan.second.size() - 1; ++j) {
                path_length += (plan.second[j + 1] - plan.second[j]).norm();
              }
              trial_proto->set_path_length(path_length);

              auto use_scaffolding_string =
                  ((use_scaffold) ? "Scaffold" : "NoScaffold");

              const auto smoothed_path = navigation::SmoothPath(
                  plan.second, scenerio.obstacle_flag, scenerio.margin);

              float smoothed_path_length = 0;
              for (size_t j = 0; j < plan.second.size() - 1; ++j) {
                smoothed_path_length +=
                    (smoothed_path[j + 1] - smoothed_path[j]).norm();
              }
              trial_proto->set_smoothed_path_length(smoothed_path_length);

              graph::util::WritePathableMultiGraphToProtoFile(
                  "RRTPath" + navigation::test::GetScenerioName(scenerio_type) +
                      use_scaffolding_string + ".proto",
                  rrt.GetScaffoldGraphs(), plan.second, smoothed_path);

              graph::util::WriteObstacleFlagToFile(
                  "DumpRRTObstacles" +
                      navigation::test::GetScenerioName(scenerio_type) +
                      use_scaffolding_string + ".proto",
                  scenerio.obstacle_flag);
            }
          }
        }
      }
    }
    navigation::test::WritePerfomanceMetrics(
        "PerformanceRRT" + navigation::test::GetScenerioName(scenerio_type) +
            ".proto",
        scenerio_proto);
  }
}

TEST(PerformanceTests, ManyIterRRT) {
  const int kNumSeeds = 10;
  const int kSeedRepetitions = 5;
  const int kSeedSets = 100;

  PositionVelocityState pvs;

  Pose2Df current_pose;
  for (int i = 0; i < 6; ++i) {
    if (i == 0) {
      current_pose.angle = 0;
      current_pose.translation.x() = 1000 * (i - 2);
      current_pose.translation.y() = 1000;
    } else {
      Pose2Df robot_pose(0, Vector2f(1000 * (i - 2), 1000));
      PositionVelocityState::RobotPositionVelocity robot(
          i, robot_pose, Pose2Df(0, Vector2f(0, 0)), robot_pose, {0, 0, 0}, 0,
          1);
      pvs.GetMutableOurTeamRobots()->InsertBack(robot);
    }
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);

  for (int seed_count = 0; seed_count < kNumSeeds; ++seed_count) {
    // Period isn't a power of 10.
    int64_t seed = 12345 + 1234567 * seed_count;
    cout << "Seed " << seed_count << ": " << seed << endl;

    for (int rep_count = 0; rep_count < kSeedRepetitions; ++rep_count) {
      cout << "Rep: " << rep_count << endl;
      FunctionTimer function_timer("Single Loop Time");

      // Set the seed for the RNG used by PRM_SSL.
      srand(seed);
      SafetyMargin margin;
      RRT rrt(ObstacleFlag::GetAll(world_state, soccer_state, 0), margin, seed,
              /* use_scaffolding = */ true);

      Vector2f goal_pos(4000, 1000);

      for (int set_count = 0; set_count < kSeedSets; ++set_count) {
        rrt.Update(ObstacleFlag::GetAll(world_state, soccer_state, 0),
                   current_pose.translation, goal_pos, margin);
        auto plan = rrt.Plan();
      }
    }
  }
}

}  // namespace rrt
}  // namespace navigation
