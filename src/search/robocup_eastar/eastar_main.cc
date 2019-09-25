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

#include <glog/logging.h>
#include <google/protobuf/stubs/common.h>
#include <signal.h>
#include <unordered_set>

#include "constants/constants.h"
#include "search/robocup_eastar/common_defines.h"
#include "search/robocup_eastar/eastar_solver.h"
#include "search/robocup_eastar/individual_planner.h"
#include "search/robocup_eastar/multiagent_data.h"
#include "search/robocup_eastar/path_collision_detector.h"
#include "search/robocup_eastar/planner_state.h"
#include "search/robocup_eastar/search_window.h"
#include "search/robocup_eastar/window_merger.h"
#include "search/robocup_logreader/logreader.h"
#include "util/helpers.h"
#include "util/serialization.h"

using search::eastar::CollidingRobot;
using search::eastar::CollisionEvent;
using search::eastar::FreeSpaceVertex;
using search::eastar::GridPath;
using search::eastar::IndividuallyPlannedDataSlice;
using search::eastar::IndividualPlansArray;
using search::eastar::JointPosition;
using search::eastar::kRoboCupEAStarMaxRobots;
using search::eastar::PlannerState;
using search::eastar::PositionsDataSlice;
using search::eastar::RobotInfo;
using search::eastar::SearchWindow;
using search::eastar::util::GridVertexToFreeSpace;
using search::eastar::operator<<;

static constexpr bool kDumpSearchPaths = false;

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

std::vector<PlannerState<kRoboCupEAStarMaxRobots>> ConstructPlannerStates(
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice,
    const std::vector<SearchWindow<kRoboCupEAStarMaxRobots>>& search_windows) {
  std::vector<PlannerState<kRoboCupEAStarMaxRobots>> planner_states;
  for (size_t i = 0; i < search_windows.size(); ++i) {
    planner_states.push_back({slice, search_windows[i]});
  }
  return planner_states;
}

void PrintPath(const PlannerState<kRoboCupEAStarMaxRobots>& planner_state) {
  for (const auto& jdp : planner_state.repaired_paths) {
    LOG(INFO) << jdp;
  }
}

void ComputeIndividualSolutionCost(
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice) {
  auto sum = 0;
  for (const auto& p : slice.individual_plans) {
    sum += p.size() - 1;
  }

  std::cout << "Individual plan costs: " << sum << '\n';
}

void ProcessPositionSlice(
    const PositionsDataSlice<kRoboCupEAStarMaxRobots>& pds,
    const size_t window_index,
    const float inflation,
    const int starting_window_radius) {
  static constexpr bool kDebug = false;
  const auto individual_plan_start = GetMonotonicTime();
  const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots> slice =
      search::eastar::GenerateIndividuallyPlannedDataSlice(pds);
  const auto individual_plan_end = GetMonotonicTime();
  std::cout << "Time for individual plans: (ms): "
            << (individual_plan_end - individual_plan_start) * 1000.0f << '\n';
  double total_runtime_ms =
      (individual_plan_end - individual_plan_start) * 1000.0f;

  const std::vector<CollisionEvent<kRoboCupEAStarMaxRobots>> collision_events =
      search::eastar::GetCollisionEvents(slice);
  if (collision_events.empty()) {
    std::cout << "No collision events found" << '\n';
    std::cout << "Total runtime (ms): " << total_runtime_ms << '\n';
    return;
  }

  std::vector<SearchWindow<kRoboCupEAStarMaxRobots>> search_windows =
      search::eastar::CollisionEventsToSearchWindows(collision_events,
                                                     starting_window_radius);
  std::vector<PlannerState<kRoboCupEAStarMaxRobots>> planner_states =
      ConstructPlannerStates(slice, search_windows);
  NP_CHECK(search_windows.size() == planner_states.size());
  if (kDumpSearchPaths) {
    search::eastar::DumpCollisionWindows(search_windows, slice, window_index);
  }

  for (size_t xstar_iteration = 0; !search::eastar::ShouldTerminateXStar(
           slice, search_windows, planner_states);
       ++xstar_iteration) {
    const auto search_windows_copy = search_windows;

    const auto search_itr_start = GetMonotonicTime();
    for (size_t i = 0; i < search_windows.size(); ++i) {
      SearchWindow<kRoboCupEAStarMaxRobots>* search_window =
          &(search_windows[i]);
      PlannerState<kRoboCupEAStarMaxRobots>* planner_state =
          &(planner_states[i]);
      std::cout << "Running X* on window of "
                << planner_state->current_start_goal.goal.position.size()
                << " agents\n";
      search::eastar::RunExpandingAStar(
          slice, search_window, planner_state, inflation);
    }

    const auto search_itr_end = GetMonotonicTime();

    if (kDumpSearchPaths) {
      search::eastar::DumpCollisionWindows(search_windows_copy,
                                           planner_states,
                                           window_index + xstar_iteration + 1);
    }

    ComputeIndividualSolutionCost(slice);
    // //     ComputeTotalSolutionCost(slice, search_windows_copy,
    // planner_states);

    if (kDebug) {
      for (const auto& ps : planner_states) {
        //       for (const auto& e : ps.repaired_paths) {
        //         LOG(INFO) << e;
        //       }
        const auto end = ps.repaired_paths.at(ps.repaired_paths.size() - 1);
        auto sum = 0;
        for (const auto& e : end.step) {
          sum += e.transit_steps;
        }
        LOG(INFO) << "Cost: " << sum << " " << end;
      }
    }

    total_runtime_ms += (search_itr_end - search_itr_start) * 1000.0;
    std::cout << "Time X* for iteration " << xstar_iteration + 1
              << " (ms): " << (search_itr_end - search_itr_start) * 1000.0
              << ", Total: " << total_runtime_ms << '\n';

    search::eastar::MergeTouchingSearchWindows(
        &search_windows, &planner_states, slice);

    search::eastar::RemoveCompletedWindows(
        &search_windows, &planner_states, slice);

    search::eastar::AddNewlyCollidingPaths(
        &search_windows, &planner_states, slice);

    // TODO(kvedder): Merge windows if needed.
    //     const auto& merge_result =
    //         MergeTouchingSearchWindows(search_windows, planner_states,
    //         slice);
    //     search_windows = merge_result.first;
    //     planner_states = merge_result.second;
  }
  std::cout << "Total runtime (ms): " << total_runtime_ms << '\n';

  for (PlannerState<kRoboCupEAStarMaxRobots>& planner_state : planner_states) {
    ClearOpenList(&planner_state);
  }

  //   LOG(INFO) << "==================";
  //   LOG(INFO) << "Final Planned Path";
  //   LOG(INFO) << "==================";
  //   for (const auto& planner_state : planner_states) {
  //     PrintPath(planner_state);
  //   }
}

void ComputeOverlap() {
  static constexpr int kMinX = -10;
  static constexpr int kMaxX = 10;
  static constexpr int kMinY = -10;
  static constexpr int kMaxY = 10;

  std::unordered_set<size_t> set;
  size_t total_count = 0;
  for (int x1 = kMinX; x1 <= kMaxX; ++x1) {
    for (int y1 = kMinY; y1 <= kMaxY; ++y1) {
      for (int x2 = kMinX; x2 <= kMaxX; ++x2) {
        for (int y2 = kMinY; y2 <= kMaxY; ++y2) {
          ::search::eastar::JointPosition<kRoboCupEAStarMaxRobots> p;
          p.push_back({x1, y1});
          p.push_back({x2, y2});
          ++total_count;
          set.insert(
              ::search::eastar::JointPositionHasher<kRoboCupEAStarMaxRobots>()(
                  p));
        }
      }
    }
  }
  LOG(INFO) << "Set count: " << set.size();
  LOG(INFO) << "Total count: " << total_count;
  LOG(INFO) << "Unique: "
            << static_cast<float>(set.size()) /
                   static_cast<float>(total_count) * 100.0f
            << "%";
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
  const int starting_window_radius = std::stoi(argv[4]);
  const int rng_seed = std::stoi(argv[5]);

  std::cout << "Running " << event_name << " with " << num_robots
            << " robots, Epsilon: " << epsilon << "\n";

  if (kDumpSearchPaths) {
    LOG(INFO) << "Files written to path: "
              << ::util::serialization::GetFullFolderPath();
  }
  const auto positions_data =
      search::logreader::GetPositionsData(num_robots, event_name, rng_seed);
  for (size_t i = 0; i < positions_data.size(); ++i) {
    ProcessPositionSlice(positions_data[i], i, epsilon, starting_window_radius);
  }
  if (kDumpSearchPaths) {
    LOG(INFO) << "Files written to path: "
              << ::util::serialization::GetFullFolderPath();
  }
}
