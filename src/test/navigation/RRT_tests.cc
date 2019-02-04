
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
#include "graph/fastmultigraph.h"
#include "graph/graph_util.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "navigation/RRT.h"

#include "navigation/scaffolding/scaffold.h"
#include "obstacles/ball_obstacle.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "util/timer.h"

using pose_2d::Pose2Df;

namespace navigation {
namespace rrt {

TEST(RRTTests, DumpGraphTest) {
  state::PositionVelocityState pvs;
  state::WorldState world_state(&pvs, team::Team::BLUE);

  {
    state::PositionVelocityState::RobotPositionVelocity world_robot(
        0, Pose2Df(0, Vector2f(0, 0)), Pose2Df(0, Vector2f(0, 0)),
        Pose2Df(0, Vector2f(0, 0)), {0, 0, 0}, 0, 0);

    pvs.GetMutableTheirTeamRobots()->InsertBack(world_robot);
  }

  obstacle::ObstacleFlag flag = obstacle::ObstacleFlag::GetAllRobots();

  obstacle::SafetyMargin safety_margin;
  RRT rrt(flag, safety_margin, /* use_scaffolding = */ true, 0.3);

  rrt.Update(flag, Vector2f(-500, 0), Vector2f(500, 0), safety_margin);

  auto plan = rrt.Plan();

  LOG(INFO) << "Final plan:";
  for (const auto& point : plan.second) {
    LOG(INFO) << point.x() << ", " << point.y();
  }
}

TEST(Scaffold, ConnectFarAwayPoint) {
  state::PositionVelocityState pvs;

  for (int i = 0; i < static_cast<int>(kMaxTeamRobots); ++i) {
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

  for (const auto* obstacle : flag) {
    LOG(INFO) << "Pose: " << obstacle->GetPose().translation.x() << ", "
              << obstacle->GetPose().translation.y();
    const graph::Graph scaffold = scaffolding::GenerateCircleScaffold(
        obstacle->GetPose(), radius, num_layers, verticies_per_layer,
        layer_offset);
    fmg.AddAdditionalGraph(scaffold);
  }

  scaffolding::InsertScaffolds(base_graph_index, flag, safety_margin, 200,
                               layer_offset, num_layers, verticies_per_layer,
                               &fmg);

  graph::util::WritePathableMultiGraphToProtoFile("ConnectFarAway.proto", fmg,
                                                  {}, {});

  graph::util::WriteObstacleFlagToFile("ConnectFarAwayObs.proto", flag);
}

}  // namespace rrt
}  // namespace navigation
