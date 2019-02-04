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

#include <fcntl.h>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "graph/graph.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"
#include "performance.pb.h"
#include "state/position_velocity_state.h"

#ifndef SRC_TEST_NAVIGATION_SCENARIO_GENERATOR_H_
#define SRC_TEST_NAVIGATION_SCENARIO_GENERATOR_H_

namespace navigation {
namespace test {

struct Scenario {
  obstacle::ObstacleFlag obstacle_flag;
  obstacle::SafetyMargin margin;
  Eigen::Vector2f start;
  Eigen::Vector2f goal;
};

enum ScenerioType {
  LINE,
  WALL,
  WALL_CLOSE,
  NARROWING,
  RANDOM,
  RANDOM_GAP,
  RANDOM_WALL,
  DENSE
};

std::vector<ScenerioType> GetScenerioTypes();

std::string GetScenerioName(const ScenerioType& scenerio_type);

Scenario GetScenerio(const ScenerioType& scenerio_type);
Scenario GetScenerio(const ScenerioType& scenerio_type,
                     const unsigned int num_robots_per_team);

Scenario GenerateLine();

Scenario GenerateWall();
Scenario GenerateWallClose();

Scenario GenerateNarrowing();

Scenario GenerateRandom();

Scenario GenerateRandom(int num_robots_per_team);

Scenario GenerateRandomGap();
Scenario GenerateRandomGap(int num_robots_per_team);

Scenario GenerateRandomWall();
Scenario GenerateRandomWall(int num_robots_per_team);

Scenario GenerateSmallSet();

// Resolution must cleanly divide into Field width and height
// Resolution must be >= 500
// Populates the input graph with a grid
void GenerateGrid(float resolution, bool connect_diagnals, graph::Graph* graph);

void WritePerfomanceMetrics(const std::string& file_path,
                            const MinuteBotsProto::Scenerio& scenerio);
}  // namespace test
}  // namespace navigation

#endif  // SRC_TEST_NAVIGATION_SCENARIO_GENERATOR_H_
