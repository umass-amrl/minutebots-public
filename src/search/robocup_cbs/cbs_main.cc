// Copyright 2018 - 2019 kvedder@umass.edu
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
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "navigation/production/collision_grid.h"
#include "obstacles/obstacle_flag.h"
#include "search/robocup_eastar/common_defines.h"
#include "search/robocup_eastar/eastar_solver.h"
#include "search/robocup_eastar/individual_planner.h"
#include "search/robocup_eastar/multiagent_data.h"
#include "search/robocup_eastar/path_collision_detector.h"
#include "search/robocup_eastar/planner_state.h"
#include "search/robocup_eastar/search_window.h"
#include "search/robocup_logreader/logreader.h"
#include "util/helpers.h"
#include "util/serialization.h"

using search::eastar::FreeSpaceVertex;
using search::eastar::GridVertex;
using search::eastar::kRoboCupEAStarMaxRobots;
using search::eastar::PositionsDataSlice;
using search::eastar::RobotInfo;
using search::eastar::util::FreeSpaceToGridVertex;
using Path = std::vector<std::vector<std::pair<int, int>>>;

void FatalSignalHandler(const int signo) {
  fprintf(stderr,
          "Received fatal signal %s, firing custom stack trace:\n",
          strsignal(signo));
  fflush(stderr);
  PrintStackTrace();
  exit(1);
}

void Init(char* argv0) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv0);
  FLAGS_stderrthreshold = 0;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk

  if (signal(SIGINT, FatalSignalHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGINT\n";
  }

  if (signal(SIGSEGV, FatalSignalHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGSEGV\n";
  }

  if (signal(SIGABRT, FatalSignalHandler) == SIG_ERR) {
    LOG(FATAL) << "Cannot trap SIGABRT\n";
  }
}

std::pair<int, int> GridVerexToIntPair(const GridVertex& v) {
  static const int x_min =
      -std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  static const int y_min =
      -std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);
  NP_CHECK_MSG(std::abs(v.x()) <= std::abs(x_min), "X outside of field!");
  NP_CHECK_MSG(
      std::abs(v.y()) <= std::abs(y_min),
      "Y outside of field! (" << v.x() << ", " << v.y() << ") vs " << y_min;);
  const std::pair<int, int> result((v.x() - x_min), (v.y() - y_min));
  NP_CHECK(result.first >= 0);
  NP_CHECK(result.second >= 0);
  return result;
}

GridVertex IntPairToGridVertex(const std::pair<int, int>& p) {
  static const int x_min =
      -std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  static const int y_min =
      -std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);
  return {p.first + x_min, p.second + y_min};
}

std::ostream& operator<<(std::ostream& os,
                         const std::vector<std::pair<int, int>>& slice) {
  os << "Path Slice: ";
  for (const std::pair<int, int>& p : slice) {
    const GridVertex v = IntPairToGridVertex(p);
    os << "(" << v.x() << ", " << v.y() << ") ";
  }
  return os;
}

std::string PositionToYAMLString(const std::pair<int, int>& p) {
  return "[" + std::to_string(p.first) + ", " + std::to_string(p.second) + "]";
}

void GenerateYAMLFile(const std::vector<std::pair<int, int>>& init_pos,
                      const std::vector<std::pair<int, int>>& goals,
                      const std::vector<std::pair<int, int>>& blocked_squares) {
  CHECK_EQ(init_pos.size(), goals.size());
  std::ofstream ofs("cbs_test_config.yaml", std::ofstream::out);
  ofs << "agents:\n";
  for (size_t i = 0; i < init_pos.size(); ++i) {
    const auto& start = init_pos.at(i);
    const auto& goal = goals.at(i);
    ofs << "-   goal: " + PositionToYAMLString(goal) +
               "\n"
               "    name: agent" +
               std::to_string(i) +
               "\n"
               "    start: " +
               PositionToYAMLString(start) + "\n";
  }
  ofs << "map:\n";

  const int map_x =
      2 * std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  const int map_y =
      2 * std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);

  ofs << "    dimensions: " + PositionToYAMLString({map_x, map_y}) +
             "\n"
             "    obstacles:\n";

  for (const auto& blocked_square : blocked_squares) {
    ofs << "    - !!python/tuple " + PositionToYAMLString(blocked_square) +
               "\n";
  }

  ofs.close();
}

void ProcessPositionSlice(
    const PositionsDataSlice<kRoboCupEAStarMaxRobots>& pds,
    const size_t window_index,
    const float inflation) {
  const navigation::production::eight_grid::CollisionGrid collision_grid =
      search::eastar::GenerateIndividuallyPlannedDataSlice(pds).collision_grid;

  NP_CHECK(!pds.obstacles.empty());

  obstacle::ObstacleFlag obstacle_flag = pds.obstacles.at(0);
  for (size_t i = 1; i < pds.obstacles.size(); ++i) {
    obstacle_flag = obstacle_flag & pds.obstacles.at(i);
  }

  const std::vector<std::vector<bool>> world_map =
      collision_grid.GenerateDenseOccupancyGrid(obstacle_flag);
  NP_CHECK(!world_map.empty());

  std::vector<std::pair<int, int>> init_pos;
  std::vector<std::pair<int, int>> goals;
  std::vector<std::pair<int, int>> blocked_squares = {{0, 0}};

  for (const RobotInfo& ri : pds.our_robots) {
    init_pos.push_back(GridVerexToIntPair(FreeSpaceToGridVertex(ri.position)));
  }

  for (const FreeSpaceVertex& v : pds.goal_positions) {
    goals.push_back(GridVerexToIntPair(FreeSpaceToGridVertex(v)));
  }

  for (size_t x = 0; x < world_map.size(); ++x) {
    for (size_t y = 0; y < world_map.at(0).size(); ++y) {
      if (world_map.at(x).at(y)) {
        blocked_squares.push_back({x, y});
      }
    }
  }

  GenerateYAMLFile(init_pos, goals, blocked_squares);
}

int main(int argc, char** argv) {
  Init(argv[0]);

  if (argc != 6) {
    LOG(FATAL) << "Usage: " << argv[0]
               << " [Num Robots] [Event Name] [Epsilon] [Starting window "
                  "radius] [RNG seed]";
  }

  const int num_robots = std::stoi(argv[1]);
  const std::string event_name = argv[2];
  const float epsilon = std::stof(argv[3]);
  //   const int starting_window_radius = std::stoi(argv[4]);
  const int rng_seed = std::stoi(argv[5]);

  const auto positions_data =
      search::logreader::GetPositionsData(num_robots, event_name, rng_seed);
  for (size_t i = 0; i < positions_data.size(); ++i) {
    ProcessPositionSlice(positions_data[i], i, epsilon);
  }
  return 0;
}
