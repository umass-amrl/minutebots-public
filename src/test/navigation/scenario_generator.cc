// Copyright 2017 - 2018 kvedder@umass.edu
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
#include "test/navigation/scenario_generator.h"

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <vector>

#include "constants/constants.h"
#include "graph/graph.h"
#include "graph/vertex.h"
#include "math/geometry.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "util/random.h"

STANDARD_USINGS;
using geometry::EuclideanDistance;
using graph::Graph;
using graph::VertexIndex;
using state::PositionVelocityState;
using pose_2d::Pose2Df;
using obstacle::ObstacleFlag;
using state::WorldState;
using state::SoccerState;

namespace navigation {
namespace test {

vector<ScenerioType> GetScenerioTypes() { return {DENSE}; }

Scenario GetScenerio(const ScenerioType& scenerio_type) {
  switch (scenerio_type) {
    case NARROWING:
      return GenerateNarrowing();
    case WALL:
      return GenerateWall();
    case WALL_CLOSE:
      return GenerateWallClose();
    case RANDOM:
      return GenerateRandom();
    case RANDOM_GAP:
      return GenerateRandomGap();
    case RANDOM_WALL:
      return GenerateRandomWall();
    case DENSE:
      return GenerateRandom(25);
    case LINE:
    default:
      return GenerateLine();
  }
}

Scenario GetScenerio(const ScenerioType& scenerio_type,
                     const unsigned int num_robots_per_team) {
  switch (scenerio_type) {
    case RANDOM:
      return GenerateRandom(num_robots_per_team);
    default:
      return GenerateLine();
  }
}

string GetScenerioName(const ScenerioType& scenerio_type) {
  switch (scenerio_type) {
    case NARROWING:
      return "Narrowing";
    case WALL:
      return "Wall";
    case DENSE:
      return "Dense";
    case LINE:
      return "Line";
    case WALL_CLOSE:
      return "WallClose";
    case RANDOM:
      return "Random";
    case RANDOM_GAP:
      return "RandomGap";
    case RANDOM_WALL:
      return "RandomWall";
    default:
      return "Unknown";
  }
}

Scenario GenerateLine() {
  PositionVelocityState pvs;

  for (int i = 1; i < 6; ++i) {
    Pose2Df robot_pose(0, Vector2f(1000 * (i - 2), 1000));
    PositionVelocityState::RobotPositionVelocity robot(
        i, robot_pose, Pose2Df(0, Vector2f(1000 * (i - 2), 1000)), robot_pose,
        {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(robot);
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);

  return {ObstacleFlag::GetAllRobotsExceptTeam(0),
          obstacle::SafetyMargin(),
          {-500, 1000},
          {4000, 1000}};
}

Scenario GenerateWall() {
  PositionVelocityState pvs;

  const std::vector<Eigen::Vector2f> positions = {
      {9999, -9999}, {1000, -200}, {1000, 200}, {600, 0}};

  Eigen::Vector2f current_pose(0, 0);
  for (int i = 0; i < 6; ++i) {
    Pose2Df robot_pose;
    if (i > 3) {
      robot_pose = Pose2Df(0, -3000, 500 * (i - 1));
    } else {
      robot_pose = Pose2Df(0, positions[i]);
    }
    PositionVelocityState::RobotPositionVelocity robot(
        i, robot_pose, Pose2Df(0, Vector2f(400 * (i - 2), 1000)), robot_pose,
        {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(robot);
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);

  return {ObstacleFlag::GetAllRobotsExceptTeam(0),
          obstacle::SafetyMargin(), current_pose, Vector2f(2000, 0)};
}

Scenario GenerateWallClose() {
  PositionVelocityState pvs;

  Vector2f current_pose(500, 400);
  for (int i = 0; i < 6; ++i) {
    Pose2Df robot_pose(0, 1000, 400 * (i - 2));
    PositionVelocityState::RobotPositionVelocity robot(
        i, robot_pose, Pose2Df(0, Vector2f(400 * (i - 2), 1000)), robot_pose,
        {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(robot);
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);

  return {ObstacleFlag::GetAllRobotsExceptTeam(0),
          obstacle::SafetyMargin(), current_pose, Vector2f(1500, 400)};
}

Scenario GenerateNarrowing() {
  PositionVelocityState pvs;

  Vector2f current_pose(200, 0);
  for (int i = 0; i < 6; ++i) {
    Pose2Df our_robot_pose(0, Vector2f(400 * (i - 2), 500 - i * 50));
    PositionVelocityState::RobotPositionVelocity our_robot(
        i, our_robot_pose, Pose2Df(0, Vector2f(1000 * (i - 2), 1000)),
        {0, 0, 0}, our_robot_pose, 0, 1);
    Pose2Df their_robot_pose(0, Vector2f(400 * (i - 2), -500 + i * 50));
    PositionVelocityState::RobotPositionVelocity their_robot(
        i, their_robot_pose, Pose2Df(0, Vector2f(1000 * (i - 2), 1000)),
        their_robot_pose, {0, 0, 0}, 0, 1);
    pvs.GetMutableOurTeamRobots()->InsertBack(our_robot);
    pvs.GetMutableTheirTeamRobots()->InsertBack(their_robot);
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);

  return {ObstacleFlag::GetAllRobots(), obstacle::SafetyMargin(), current_pose,
          Vector2f(2000, 1000)};
}

Scenario GenerateRandom() { return GenerateRandom(kMaxTeamRobots); }

Scenario GenerateRandom(int num_robots_per_team) {
  PositionVelocityState pvs;
  util_random::Random random;

  unsigned int num_robots = num_robots_per_team;
  if (num_robots > kMaxTeamRobots) num_robots = kMaxTeamRobots;

  Vector2f start_position(0, 0);
  Vector2f goal_position(2000, 0);

  for (unsigned int i = 0; i < num_robots; i++) {
    Vector2f our_robot_position;
    Vector2f their_robot_position;

    our_robot_position.x() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
    our_robot_position.y() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));

    while (EuclideanDistance(our_robot_position, start_position) <
               2 * kRobotRadius + 50 &&
           EuclideanDistance(our_robot_position, goal_position) <
               2 * kRobotRadius + 5) {
      our_robot_position.x() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
      our_robot_position.y() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));
    }

    their_robot_position.x() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
    their_robot_position.y() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));

    while (EuclideanDistance(their_robot_position, start_position) <
               2 * kRobotRadius + 50 &&
           EuclideanDistance(their_robot_position, goal_position) <
               2 * kRobotRadius + 5) {
      their_robot_position.x() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
      their_robot_position.y() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));
    }

    Pose2Df our_robot_pose(0, our_robot_position);
    PositionVelocityState::RobotPositionVelocity our_robot(
        i, our_robot_pose, Pose2Df(0, 0, 0), our_robot_pose, {0, 0, 0}, 0, 1);
    Pose2Df their_robot_pose(0, their_robot_position);
    PositionVelocityState::RobotPositionVelocity their_robot(
        i, their_robot_pose, Pose2Df(0, 0, 0), their_robot_pose, {0, 0, 0}, 0,
        1);
    pvs.GetMutableOurTeamRobots()->InsertBack(our_robot);
    pvs.GetMutableTheirTeamRobots()->InsertBack(their_robot);
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);
  return {ObstacleFlag::GetAllRobots(), obstacle::SafetyMargin(),
          start_position, goal_position};
}

Scenario GenerateRandomGap() { return GenerateRandomGap(kMaxTeamRobots); }

Scenario GenerateRandomGap(int num_robots_per_team) {
  PositionVelocityState pvs;
  util_random::Random random;

  unsigned int num_robots = num_robots_per_team;
  if (num_robots > kMaxTeamRobots) num_robots = kMaxTeamRobots;

  Vector2f start_position(0, 0);
  Vector2f goal_position(2000, 0);

  Pose2Df our_robot_pose_1(
      0, Vector2f(200, 1.05 * (kRobotRadius + kDefaultSafetyMargin)));
  PositionVelocityState::RobotPositionVelocity our_robot(
      0, our_robot_pose_1, Pose2Df(0, 0, 0), our_robot_pose_1, {0, 0, 0}, 0, 1);
  Pose2Df their_robot_pose_2(
      0, Vector2f(200, -1.05 * (kRobotRadius + kDefaultSafetyMargin)));
  PositionVelocityState::RobotPositionVelocity their_robot(
    0, their_robot_pose_2, Pose2Df(0, 0, 0), their_robot_pose_2, {0, 0, 0}, 0,
    1);
  pvs.GetMutableOurTeamRobots()->InsertBack(our_robot);
  pvs.GetMutableTheirTeamRobots()->InsertBack(their_robot);

  for (unsigned int i = 1; i < num_robots - 2; i++) {
    Vector2f our_robot_position;
    Vector2f their_robot_position;

    our_robot_position.x() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
    our_robot_position.y() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));

    while (EuclideanDistance(our_robot_position, start_position) <
               2 * kRobotRadius + 50 &&
           EuclideanDistance(our_robot_position, goal_position) <
               2 * kRobotRadius + 5) {
      our_robot_position.x() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
      our_robot_position.y() = static_cast<float>(
          random.UniformRandom(-kHalfFieldWidth, kHalfFieldWidth));
    }

    their_robot_position.x() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
    their_robot_position.y() = static_cast<float>(
        random.UniformRandom(-kHalfFieldWidth, kHalfFieldWidth));

    while (EuclideanDistance(their_robot_position, start_position) <
               2 * kRobotRadius + 50 &&
           EuclideanDistance(their_robot_position, goal_position) <
               2 * kRobotRadius + 5) {
      their_robot_position.x() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
      their_robot_position.y() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));
    }

    Pose2Df our_robot_pose(0, our_robot_position);
    PositionVelocityState::RobotPositionVelocity our_robot(
        i, our_robot_pose, Pose2Df(0, 0, 0), our_robot_pose, {0, 0, 0} , 0, 1);
    Pose2Df their_robot_pose(0, their_robot_position);
    PositionVelocityState::RobotPositionVelocity their_robot(
        i, their_robot_pose, Pose2Df(0, 0, 0), their_robot_pose, {0, 0, 0}, 0,
        1);
    pvs.GetMutableOurTeamRobots()->InsertBack(our_robot);
    pvs.GetMutableTheirTeamRobots()->InsertBack(their_robot);
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);
  return {ObstacleFlag::GetAllRobots(), obstacle::SafetyMargin(),
          start_position, goal_position};
}

Scenario GenerateRandomWall() { return GenerateRandomWall(kMaxTeamRobots); }

Scenario GenerateRandomWall(int num_robots_per_team) {
  PositionVelocityState pvs;
  util_random::Random random;

  unsigned int num_robots = num_robots_per_team;
  if (num_robots > kMaxTeamRobots) num_robots = kMaxTeamRobots;

  Vector2f start_position(0, 0);
  Vector2f goal_position(2000, 0);

  Pose2Df our_robot_pose_1(0, Vector2f(200, (kDefaultSafetyMargin)));
  PositionVelocityState::RobotPositionVelocity our_robot_1(
      0, our_robot_pose_1, Pose2Df(0, 0, 0), our_robot_pose_1, {0, 0, 0}, 0, 1);
  Pose2Df their_robot_pose_1(0, Vector2f(200, -(kDefaultSafetyMargin)));
  PositionVelocityState::RobotPositionVelocity their_robot_1(
      0, their_robot_pose_1, Pose2Df(0, 0, 0), their_robot_pose_1, {0, 0, 0}, 0,
      1);
  pvs.GetMutableOurTeamRobots()->InsertBack(our_robot_1);
  pvs.GetMutableTheirTeamRobots()->InsertBack(their_robot_1);

  Pose2Df our_robot_pose_2(0, Vector2f(200, 2 * (kDefaultSafetyMargin)));
  PositionVelocityState::RobotPositionVelocity our_robot_2(
      0, our_robot_pose_2, Pose2Df(0, 0, 0), our_robot_pose_2, {0, 0, 0}, 0, 1);
  Pose2Df their_robot_pose_2(
      0, Vector2f(200, -2 * (kRobotRadius + kDefaultSafetyMargin)));
  PositionVelocityState::RobotPositionVelocity their_robot_2(
      0, their_robot_pose_2, Pose2Df(0, 0, 0), their_robot_pose_2, {0, 0, 0},
      0, 1);
  pvs.GetMutableOurTeamRobots()->InsertBack(our_robot_2);
  pvs.GetMutableTheirTeamRobots()->InsertBack(their_robot_2);

  for (unsigned int i = 2; i < num_robots; i++) {
    Vector2f our_robot_position;
    Vector2f their_robot_position;

    our_robot_position.x() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
    our_robot_position.y() = static_cast<float>(
        random.UniformRandom(-kHalfFieldWidth, kHalfFieldWidth));

    while (EuclideanDistance(our_robot_position, start_position) <
               2 * kRobotRadius + 50 &&
           EuclideanDistance(our_robot_position, goal_position) <
               2 * kRobotRadius + 5) {
      our_robot_position.x() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
      our_robot_position.y() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));
    }

    their_robot_position.x() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
    their_robot_position.y() =
        static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));

    while (EuclideanDistance(their_robot_position, start_position) <
               2 * kRobotRadius + 50 &&
           EuclideanDistance(their_robot_position, goal_position) <
               2 * kRobotRadius + 5) {
      their_robot_position.x() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldLength));
      their_robot_position.y() =
          static_cast<float>(random.UniformRandom(-200, kHalfFieldWidth));
    }

    Pose2Df our_robot_pose(0, our_robot_position);
    PositionVelocityState::RobotPositionVelocity our_robot(
        i, our_robot_pose, Pose2Df(0, 0, 0), our_robot_pose, {0, 0, 0}, 0, 1);
    Pose2Df their_robot_pose(0, their_robot_position);
    PositionVelocityState::RobotPositionVelocity their_robot(
      i, their_robot_pose, Pose2Df(0, 0, 0), their_robot_pose, {0, 0, 0}, 0,
      1);
    pvs.GetMutableOurTeamRobots()->InsertBack(our_robot);
    pvs.GetMutableTheirTeamRobots()->InsertBack(their_robot);
  }

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);
  return {ObstacleFlag::GetAllRobots(), obstacle::SafetyMargin(),
          start_position, goal_position};
}

int CreateOrEraseFileForWrite(const std::string& file_path) {
  mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
  int file_descriptor =
      open(file_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, mode);
  if (file_descriptor < 0) {
    LOG(FATAL) << "Error opening file " << file_path;
  }
  return file_descriptor;
}

void GenerateGrid(float resolution, bool connect_diagnals,
                  graph::Graph* graph) {
  // Calculate the total number of vertices and vertices per row for the input
  // resolution.
  unsigned int vertices_per_column =
      static_cast<unsigned int>(field_dimensions::kFieldWidth / resolution);
  unsigned int vertices_per_row =
      static_cast<unsigned int>(field_dimensions::kFieldLength / resolution);
  unsigned int num_total_vertices = vertices_per_column * vertices_per_row;

  // Bottom left is 0, 0
  int current_row = 0;
  int current_column = 0;

  vector<VertexIndex> vertex_indices;

  for (unsigned int i = 0; i < num_total_vertices; i++) {
    Vector2f current_position(current_column * resolution - kFieldXMax,
                              current_row * resolution - kFieldYMax);

    VertexIndex current_index = graph->AddVertex(current_position);
    vertex_indices.push_back(current_index);

    current_column++;
    if (current_column % vertices_per_row == 0) {
      current_column = 0;
      current_row++;
    }
  }

  for (unsigned int i = 0; i < num_total_vertices; i++) {
    Vector2f current_position(current_column * resolution - kFieldXMax,
                              current_row * resolution - kFieldYMax);
    VertexIndex current_index = vertex_indices[i];

    // Add right
    if ((i + 1) % vertices_per_row > 0 || i == 0) {
      Vector2f right_position(current_position);
      right_position.x() += resolution;
      graph->AddEdge(current_index, vertex_indices[i + 1],
                     EuclideanDistance(current_position, right_position));

      // Add above right
      if (i + vertices_per_row < num_total_vertices && connect_diagnals) {
        right_position.y() += resolution;

        graph->AddEdge(current_index, vertex_indices[i + vertices_per_row + 1],
                       EuclideanDistance(current_position, right_position));
      }

      // Add below right
      if (static_cast<int>(i - vertices_per_row) >= 0 && connect_diagnals) {
        right_position.y() -= 2 * resolution;

        graph->AddEdge(current_index, vertex_indices[i - vertices_per_row + 1],
                       EuclideanDistance(current_position, right_position));
      }
    }

    // Add above
    if (i + vertices_per_row < num_total_vertices) {
      Vector2f above_position(current_position);
      above_position.y() += resolution;

      graph->AddEdge(current_index, vertex_indices[i + vertices_per_row],
                     EuclideanDistance(current_position, above_position));
    }

    current_column++;
    if (current_column % vertices_per_row == 0) {
      current_column = 0;
      current_row++;
    }
  }
}

Scenario GenerateSmallSet() {
  PositionVelocityState pvs;

  Vector2f current_pose(0, 0);

  Pose2Df robot_pose1(0, 500, 10);
  PositionVelocityState::RobotPositionVelocity robot1(
    1, robot_pose1, Pose2Df(0, 0, 0), robot_pose1, {0, 0, 0}, 0, 1);
  pvs.GetMutableOurTeamRobots()->InsertBack(robot1);

  Pose2Df robot_pose2(0, 1000, 10);
  PositionVelocityState::RobotPositionVelocity robot2(
    1, robot_pose2, Pose2Df(0, 0, 0), robot_pose2, {0, 0, 0}, 0, 1);
  pvs.GetMutableOurTeamRobots()->InsertBack(robot2);

  Pose2Df robot_pose3(0, 1500, 10);
  PositionVelocityState::RobotPositionVelocity robot3(
    1, robot_pose3, Pose2Df(0, 0, 0), robot_pose3, {0, 0, 0}, 0, 1);
  pvs.GetMutableOurTeamRobots()->InsertBack(robot3);

  //   Pose2Df robot_pose4(0, 1500, -500);
  //   PositionVelocityState::RobotPositionVelocity robot4(
  //     1, robot_pose4, Pose2Df(0, 0, 0), robot_pose4, 0, 1);
  //   pvs.GetMutableOurTeamRobots()->InsertBack(robot4);

  WorldState world_state(&pvs, team::Team::YELLOW);
  SoccerState soccer_state(world_state);

  return {ObstacleFlag::GetAllRobots(), obstacle::SafetyMargin(), current_pose,
          Vector2f(2000, 0)};
}

void WritePerfomanceMetrics(const string& file_path,
                            const MinuteBotsProto::Scenerio& scenerio) {
  int file_descriptor = CreateOrEraseFileForWrite(file_path);

  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);

  if (!google::protobuf::TextFormat::Print(scenerio, &file_output)) {
    LOG(ERROR) << "Failed to write graph proto to " << file_path;
  }
}

}  // namespace test
}  // namespace navigation
