// Copyright 2018 kvedder@umass.edu
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
#include <chrono>
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

#include "third_party/mstar/grid_planning.hpp"
#include "third_party/mstar/grid_policy.hpp"
#include "third_party/mstar/mstar_type_defs.hpp"
#include "third_party/mstar/mstar_utils.hpp"
#include "third_party/mstar/od_mstar.hpp"

using mstar::Clock;
using mstar::find_grid_path;
using mstar::grid_policy_ptr;
using mstar::ODColChecker;
using mstar::OdCoord;
using mstar::OdMstar;
using mstar::OdPath;
using mstar::Policy;
using mstar::RobCoord;
using mstar::SimpleGraphODColCheck;
using mstar::time_point;
using search::eastar::FreeSpaceVertex;
using search::eastar::GridVertex;
using search::eastar::kRoboCupEAStarMaxRobots;
using search::eastar::PositionsDataSlice;
using search::eastar::RobotInfo;
using search::eastar::util::FreeSpaceToGridVertex;
using Path = std::vector<std::vector<std::pair<int, int>>>;

void FatalSignalHandler(const int signo) {
  fprintf(stderr, "Received fatal signal %s, firing custom stack trace:\n",
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
  NP_CHECK_MSG(std::abs(v.y()) <= std::abs(y_min), "Y outside of field! ("
                                                       << v.x() << ", " << v.y()
                                                       << ") vs " << y_min;);
  const std::pair<int, int> result((v.x() - x_min), (v.y() - y_min));
  NP_CHECK(result.first >= 0);
  NP_CHECK(result.second >= 0);
  return result;
}

RobCoord GridVerexToRobCoord(const GridVertex& v) {
  const std::pair<int, int> int_pair = GridVerexToIntPair(v);
  NP_CHECK(int_pair.first >= 0);
  NP_CHECK(int_pair.second >= 0);
  const int columns = static_cast<int>(
      std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize) * 2);
  return int_pair.first * columns + int_pair.second;
}

GridVertex RobCoordToGridVertex(const RobCoord& rob_coord) {
  const int columns = static_cast<int>(
      std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize) * 2);
  const int int_pair_first = rob_coord / columns;
  const int int_pair_second = rob_coord % columns;
  static const int x_min =
      -std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  static const int y_min =
      -std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);
  return {(int_pair_first + x_min), (int_pair_second + y_min)};
}

GridVertex IntPairToGridVertex(const std::pair<int, int>& p) {
  static const int x_min =
      -std::ceil(field_dimensions::kHalfFieldLength / kEightGridSquareSize);
  static const int y_min =
      -std::ceil(field_dimensions::kHalfFieldWidth / kEightGridSquareSize);
  return {p.first + x_min, p.second + y_min};
}

std::ostream& operator<<(std::ostream& os, const OdCoord& coord) {
  os << "OdCoord: ";
  for (const RobCoord& c : coord.coord) {
    const GridVertex v = RobCoordToGridVertex(c);
    os << "(" << v.x() << ", " << v.y() << ") ";
  }
  return os;
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

void ProcessPositionSlice(
    const PositionsDataSlice<kRoboCupEAStarMaxRobots>& pds,
    const size_t window_index, const float inflation) {
  const navigation::production::eight_grid::CollisionGrid collision_grid =
      search::eastar::GenerateIndividuallyPlannedDataSlice(pds).collision_grid;
  // Intended to force rM*
  time_point t = std::chrono::system_clock::now();
  t += Clock::duration(std::chrono::seconds(10));

  NP_CHECK(!pds.obstacles.empty());

  obstacle::ObstacleFlag obstacle_flag = pds.obstacles.at(0);
  for (size_t i = 1; i < pds.obstacles.size(); ++i) {
    obstacle_flag = obstacle_flag & pds.obstacles.at(i);
  }

  const std::vector<std::vector<bool>> world_map =
      collision_grid.GenerateDenseOccupancyGrid(obstacle_flag);
  NP_CHECK(!world_map.empty());

  std::vector<std::pair<int, int>> init_pos;
  //   {
  //       {
  //       26,26},{21,3},{27,6},{17,28},{4,18},{23,3},{24,26},{4,20},{14,15},{27,27},{31,20},{22,31},{30,20},{4,28},{10,19},{5,12},{14,1},{31,17},{20,0},{12,24},{21,28},{24,1},{1,28},{15,8},{5,26},{29,21},{2,25},{13,28},{30,30},{27,7},{28,23},{26,1},{13,8},{15,22},{31,15},{11,18},{12,13},{6,5},{2,27},{29,17},{9,4},{4,2},{24,28},{3,6},{10,22},{13,21},{14,25},{18,16},{17,8},{17,19},{22,15},{12,14},{31,29},{10,5},{2,5},{7,7},{16,4},{3,14},{19,28},{3,4},{12,4},{30,11},{26,13},{15,18},{17,16},{13,14},{30,25},{17,26},{11,8},{24,27},{2,21},{22,19},{8,26},{31,18},{23,26},{9,16},{3,2},{31,27},{6,11},{31,7},{22,28},{3,1},{25,2},{23,20},{23,17},{17,4},{17,7},{11,7},{8,12},{8,3}};
  std::vector<std::pair<int, int>> goals;
  //     {
  //       {29,0},{7,25},{27,11},{17,3},{9,3},{22,16},{5,7},{19,15},{1,7},{2,21},{15,17},{3,8},{14,10},{8,16},{18,21},{9,2},{10,27},{12,6},{23,12},{11,19},{23,10},{12,15},{23,16},{8,28},{14,8},{7,13},{12,12},{0,20},{20,24},{19,18},{7,2},{10,26},{31,9},{7,11},{20,13},{16,8},{21,19},{4,5},{26,11},{9,5},{25,1},{11,26},{6,14},{6,24},{27,21},{27,14},{18,19},{4,14},{20,9},{25,2},{9,10},{4,24},{15,26},{8,14},{26,21},{18,7},{25,0},{22,10},{4,31},{3,12},{27,6},{31,7},{15,5},{20,15},{11,14},{19,3},{23,24},{30,16},{23,7},{17,7},{26,23},{22,23},{10,22},{15,16},{6,21},{20,7},{18,30},{16,30},{21,17},{16,15},{15,27},{29,19},{6,16},{27,8},{0,15},{15,23},{5,25},{9,4},{14,27},{23,14}};

  for (const RobotInfo& ri : pds.our_robots) {
    init_pos.push_back(GridVerexToIntPair(FreeSpaceToGridVertex(ri.position)));
  }

  for (const FreeSpaceVertex& v : pds.goal_positions) {
    goals.push_back(GridVerexToIntPair(FreeSpaceToGridVertex(v)));
  }

  const auto planning_start = GetMonotonicTime();
  const Path path =
      find_grid_path(world_map, init_pos, goals, inflation, 100000);
  const auto planning_end = GetMonotonicTime();
  std::cout << "Planning time (ms): " << (planning_end - planning_start) * 1000
            << '\n';
  //   for (const std::vector<std::pair<int, int>>& path_slice : path) {
  //     LOG(INFO) << path_slice;
  //   }
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
