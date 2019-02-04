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
#include "graph/graph.h"
#include "graph/graph_util.h"
#include "math/poses_2d.h"
#include "navigation/PRM.h"
#include "navigation/RRT.h"
#include "navigation/navigation_util.h"
#include "navigation/scaffolding/scaffold.h"
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

TEST(FixedSeedTest, Example) {
  logger::Logger logger;
  auto scenerio = navigation::test::GenerateWall();
  size_t seed = 27;
  PRM prm(scenerio.obstacle_flag, scenerio.margin, 1000 - 3 * 12 * 5, true, 3,
          12, 40, seed);

  prm.Update(scenerio.obstacle_flag, scenerio.start, scenerio.goal, &logger);
  const auto result = prm.Plan(&logger);
  bool found_path = result.first;
  vector<Vector2f> waypoints = result.second;
  if (!found_path) {
    LOG(ERROR) << "No path found!";
  }

  float len = 0;
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    len += (waypoints[i] - waypoints[i + 1]).norm();
  }

  LOG(WARNING) << len;

  const auto smoothed_path = waypoints;

  graph::util::WritePathableMultiGraphToProtoFile(
      "JoydeepsWall.proto", prm.GetGraph(), waypoints, smoothed_path);

  graph::util::WriteObstacleFlagToFile("JoydeepsWallObs.proto",
                                       scenerio.obstacle_flag);
}

TEST(FixedSeedGridTest, Example) {
  logger::Logger logger;
  graph::Graph graph;
  navigation::test::GenerateGrid(300, true, &graph);

  auto scenerio = navigation::test::GenerateRandomWall();
  size_t seed = 524752;

  PRM prm(graph, scenerio.obstacle_flag, scenerio.margin, true, 3, 12, 40,
          seed);

  prm.Update(scenerio.obstacle_flag, scenerio.start, scenerio.goal, &logger);
  const auto result = prm.Plan(&logger);
  //   const bool found_path = result.first;
  const vector<Vector2f> waypoints = result.second;
  float len = 0;
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    len += (waypoints[i] - waypoints[i + 1]).norm();
  }

  const auto smoothed_path = waypoints;

  graph::util::WritePathableMultiGraphToProtoFile(
      "GridExample.proto", prm.GetGraph(), waypoints, smoothed_path);

  graph::util::WriteObstacleFlagToFile("GridExampleObs.proto",
                                       scenerio.obstacle_flag);
}

TEST(Scaffold, MovePerformance) {
  state::PositionVelocityState pvs;

  for (int i = 0; i < 1; ++i) {
    Pose2Df robot_pose(0, 1000, 400 * (i - 2));
    state::PositionVelocityState::RobotPositionVelocity robot(
        i, robot_pose, Pose2Df(0, Vector2f(400 * (i - 2), 1000)), robot_pose,
        {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(robot);
  }

  state::WorldState world_state(&pvs, team::Team::YELLOW);
  state::SoccerState soccer_state(world_state);

  obstacle::ObstacleFlag flag =
      obstacle::ObstacleFlag::GetAllRobotsExceptTeam(0);
  obstacle::SafetyMargin safety_margin;

  graph::Graph base_graph;
  base_graph.AddVertex(Vector2f(-300, 0));

  graph::GraphIndex base_graph_index;
  graph::multi_graph::FastMultiGraph fmg(base_graph, &base_graph_index);

  const float radius = 200;
  const int num_layers = 1;
  const int verticies_per_layer = 16;
  const int layer_offset = 40;

  graph::GraphIndex add_graph = 0;

  for (const auto* obstacle : flag) {
    LOG(INFO) << "Pose: " << obstacle->GetPose().translation.x() << ", "
              << obstacle->GetPose().translation.y();
    const graph::Graph scaffold = scaffolding::GenerateCircleScaffold(
        obstacle->GetPose(), radius, num_layers, verticies_per_layer,
        layer_offset);
    add_graph = fmg.AddAdditionalGraph(scaffold);
  }

  const auto t_start_make = GetMonotonicTime();

  for (size_t i = 0; i < 100000; ++i) {
    *fmg.GetMutableGraph(add_graph) = scaffolding::GenerateCircleScaffold(
        Pose2Df(0, 0, 0), radius, num_layers, verticies_per_layer,
        layer_offset);
  }
  const auto t_end_make = GetMonotonicTime();

  LOG(WARNING) << "Generate Scaffold: " << (t_end_make - t_start_make) / 100000;

  const auto t_start_move = GetMonotonicTime();

  for (int i = 0; i < 100000; ++i) {
    *fmg.GetMutableGraph(add_graph) = scaffolding::GenerateCircleScaffold(
        Pose2Df(i * 0.1f + 1, i * 0.0001f, i * 0.0001f), radius, num_layers,
        verticies_per_layer, layer_offset);
  }

  const auto t_end_move = GetMonotonicTime();

  LOG(WARNING) << "Update Scaffold: " << (t_end_move - t_start_move) / 100000;

  scaffolding::InsertScaffolds(base_graph_index, flag, safety_margin, 200,
                               layer_offset, num_layers, verticies_per_layer,
                               &fmg);
}

TEST(QualityTests, DumpPRMPathReal) {
  logger::Logger logger;
  constexpr auto kNumSeeds = 30u;
  constexpr auto kNumPerformanceTrials = 30u;

  // Locked
  constexpr auto kMinScaffoldLayers = 3;
  constexpr auto kMaxScaffoldLayers = 3;

  // Locked
  constexpr auto kMinVerticiesPerLayer = 12;
  constexpr auto kMaxVerticiesPerLayer = 12;

  // Locked
  constexpr auto kMinLayerOffset = 50.0f;
  constexpr auto kMaxLayerOffset = 50.0f;
  constexpr auto kLayerOffsetStep = 20.0f;

  constexpr auto kMinNumSamples = 1000;
  constexpr auto kMaxNumSamples = 2800;
  constexpr auto kNumSamplesStep = 200;

  // Locked
  constexpr float kMinConnectRadius = 500;
  constexpr float kMaxConnectRadius = 500.001;
  constexpr float kConnectRadiusStep = 200;

  for (const auto& scenerio_type : navigation::test::GetScenerioTypes()) {
    LOG(WARNING) << navigation::test::GetScenerioName(scenerio_type);
    MinuteBotsProto::Scenerio scenerio_proto;
    scenerio_proto.set_scenerio_name(
        navigation::test::GetScenerioName(scenerio_type));

    for (int total_vertices = kMinNumSamples; total_vertices <= kMaxNumSamples;
         total_vertices += kNumSamplesStep) {
      LOG(WARNING) << "Base samples: " << total_vertices;

      for (float connect_radius = kMinConnectRadius;
           connect_radius <= kMaxConnectRadius;
           connect_radius += kConnectRadiusStep) {
        LOG(WARNING) << "    Connect radius: " << connect_radius;
        for (float layer_offset = kMinLayerOffset;
             layer_offset <= kMaxLayerOffset;
             layer_offset += kLayerOffsetStep) {
          for (int verts_per_layer = kMinVerticiesPerLayer;
               verts_per_layer <= kMaxVerticiesPerLayer; ++verts_per_layer) {
            LOG(WARNING) << "        Verts per layer: " << verts_per_layer;
            for (int scaffold_layers = kMinScaffoldLayers;
                 scaffold_layers <= kMaxScaffoldLayers; ++scaffold_layers) {
              LOG(WARNING) << "            Scaffold layers: "
                           << scaffold_layers;
              auto* configuration_proto =
                  scenerio_proto.add_configuration_list();
              auto* prm_config = configuration_proto->mutable_prm_config();
              prm_config->set_num_layers(scaffold_layers);
              prm_config->set_verticies_per_layer(verts_per_layer);
              prm_config->set_layer_offset(layer_offset);
              prm_config->set_total_verticies(total_vertices);
              prm_config->set_connect_radius(connect_radius);
              LOG(FATAL)
                  << "Just an FYI, connect radius is now a hardcoded constant!";

              for (size_t scaffold_counter = 0; scaffold_counter < 2;
                   ++scaffold_counter) {
                bool use_scaffold = (scaffold_counter == 0);

                for (size_t i = 0; i < kNumSeeds; ++i) {
                  const uint64_t seed = i * 32797;
                  MinuteBotsProto::Trial* trial_proto =
                      configuration_proto->add_trial_list();
                  trial_proto->set_seed(seed);
                  trial_proto->set_scaffold(use_scaffold);
                  for (size_t trial = 0; trial < kNumPerformanceTrials;
                       ++trial) {
                    auto scenerio =
                        navigation::test::GetScenerio(scenerio_type);

                    int num_obstacles = 0;
                    for (const auto* o : scenerio.obstacle_flag) {
                      LOG(INFO) << o->GetRadius();
                      ++num_obstacles;
                    }

                    const auto samples = std::max(
                        0, ((use_scaffold) ? total_vertices
                                           : total_vertices + (scaffold_layers *
                                                               verts_per_layer *
                                                               num_obstacles)));

                    auto setup_start = GetMonotonicTime();

                    PRM prm(scenerio.obstacle_flag, scenerio.margin, samples,
                            use_scaffold, scaffold_layers, verts_per_layer,
                            layer_offset, seed);

                    auto setup_end = GetMonotonicTime();

                    trial_proto->add_setup_times_list(setup_end - setup_start);

                    auto start_time = GetMonotonicTime();

                    prm.Update(scenerio.obstacle_flag, scenerio.start,
                               scenerio.goal, &logger);
                    auto update_end_time = GetMonotonicTime();
                    const auto result = prm.Plan(&logger);
                    const bool found_path = result.first;
                    const vector<Vector2f> waypoints = result.second;

                    auto end_time = GetMonotonicTime();

                    trial_proto->add_trial_times_list(end_time - start_time);
                    trial_proto->add_update_times_list(update_end_time -
                                                       start_time);
                    trial_proto->set_path_found(found_path);
                    prm_config->set_num_obstacles(num_obstacles);

                    if (trial == (kNumPerformanceTrials - 1)) {
                      const auto& fmg = prm.GetGraph();

                      int edges = 0;
                      for (const auto& g : fmg.GetGraphs()) {
                        edges += g.GetNumEdges();
                      }
                      edges += fmg.GetInterGraphEdges().size();

                      prm_config->set_num_edges(edges);

                      trial_proto->set_path_found(found_path);
                      trial_proto->set_samples(samples);
                      trial_proto->set_path_distance(
                          (scenerio.goal - scenerio.start).norm());
                      float path_length = 0;
                      for (size_t j = 0; j < waypoints.size() - 1; ++j) {
                        path_length += (waypoints[j + 1] - waypoints[j]).norm();
                      }
                      trial_proto->set_path_length(path_length);

                      //                       auto use_scaffolding_string =
                      //                           ((use_scaffold) ? "Scaffold"
                      //                           : "NoScaffold");

                      const auto smoothed_path = waypoints;

                      float smoothed_path_length = 0;
                      for (size_t j = 0; j < waypoints.size() - 1; ++j) {
                        smoothed_path_length +=
                            (smoothed_path[j + 1] - smoothed_path[j]).norm();
                      }
                      trial_proto->set_smoothed_path_length(
                          smoothed_path_length);

                      //                       graph::util::WritePathableMultiGraphToProtoFile(
                      //                           "PRM3RadiusSweepPath" +
                      //                               navigation::test::GetScenerioName(scenerio_type)
                      //                               +
                      //                               use_scaffolding_string +
                      //                               ".proto",
                      //                           prm.GetGraph(), waypoints,
                      //                           smoothed_path);
                      //
                      //                       graph::util::WriteObstacleFlagToFile(
                      //                           "DumpPRM3RadiusSweepObstacles"
                      //                           +
                      //                               navigation::test::GetScenerioName(scenerio_type)
                      //                               +
                      //                               use_scaffolding_string +
                      //                               ".proto",
                      //                           scenerio.obstacle_flag);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    navigation::test::WritePerfomanceMetrics(
        "PerformancePRM9Dense" +
            navigation::test::GetScenerioName(scenerio_type) + ".proto",
        scenerio_proto);
  }
}

TEST(QualityTests, DumpPRMPathFake) {
  LOG(WARNING) << "Starting test!";
  const int64_t kSeed = 32797;
  logger::Logger logger;

  auto scenerio = navigation::test::GenerateLine();

  // Set the seed for the RNG used by PRM_SSL.
  srand(kSeed);

  PRM prm(scenerio.obstacle_flag, scenerio.margin, 2000, true, 1, 16, 4, 0);

  prm.Update(scenerio.obstacle_flag, scenerio.start, scenerio.goal, &logger);
  const auto result = prm.Plan(&logger);
  //   const bool found_path = result.first;
  const vector<Vector2f> waypoints = result.second;

  const auto smoothed_path = waypoints;

  graph::util::WritePathableMultiGraphToProtoFile(
      "PRMPath.proto", prm.GetGraph(), waypoints, smoothed_path);

  graph::util::WriteObstacleFlagToFile("DumpPRMObstacles.proto",
                                       scenerio.obstacle_flag);
}

TEST(PerformanceTests, ManyIterPRM) {
  const int kNumSeeds = 10;
  const int kSeedRepetitions = 5;
  const int kSeedSets = 100;
  logger::Logger logger;

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
      PRM prm(ObstacleFlag::GetAll(world_state, soccer_state, 0), margin, 2000,
              false, 0, 0, 0, 0);

      Vector2f goal_pos(4000, 1000);

      for (int set_count = 0; set_count < kSeedSets; ++set_count) {
        prm.Update(ObstacleFlag::GetAll(world_state, soccer_state, 0),
                   current_pose.translation, goal_pos, &logger);
        const auto result = prm.Plan(&logger);
        //         const bool found_path = result.first;
        const vector<Vector2f> waypoints = result.second;
      }
    }
  }
}
}  // namespace navigation
