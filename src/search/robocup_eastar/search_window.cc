// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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

#include "search/robocup_eastar/search_window.h"

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <fstream>
#include <string>
#include <vector>

#include "eastar.pb.h"
#include "search/robocup_eastar/common_defines.h"
#include "search/robocup_eastar/path_collision_detector.h"
#include "search/robocup_eastar/planner_state.h"
#include "util/serialization.h"

namespace search {
namespace eastar {

template <size_t kMaxRobotCount>
const std::vector<SearchWindow<kMaxRobotCount>> CollisionEventsToSearchWindows(
    const std::vector<CollisionEvent<kMaxRobotCount>>& collision_events,
    const int search_window_radius) {
  NP_CHECK(search_window_radius > 0);
  // Distance between event centers, where in or below that distance the two
  // event centers will be merged.
  static const GridVertex delta(-search_window_radius, search_window_radius);
  std::vector<SearchWindow<kMaxRobotCount>> search_windows;
  for (const CollisionEvent<kMaxRobotCount>& collision_event :
       collision_events) {
    const WindowBox window_box(collision_event.collision_center + delta,
                               collision_event.collision_center - delta,
                               search_window_radius);
    search_windows.push_back({window_box, collision_event.colliding_robots});
  }
  return search_windows;
}

void GridVertexToFreeSpaceProto(
    const GridVertex& grid_vertex,
    MinuteBotsProto::EAstar::FreeSpaceVertexProto* ptr) {
  const FreeSpaceVertex f = util::GridVertexToFreeSpace(grid_vertex);
  ptr->set_x(f.x());
  ptr->set_y(f.y());
}

void WindowBoxToWindowBoxProto(
    const WindowBox& box, MinuteBotsProto::EAstar::WindowBoxProto* box_proto) {
  GridVertexToFreeSpaceProto(box.upper_left, box_proto->mutable_upper_left());
  GridVertexToFreeSpaceProto(box.lower_right, box_proto->mutable_lower_right());
}

void GridPathToFreeSpacePathProto(
    const GridPath& grid_path,
    MinuteBotsProto::EAstar::FreeSpacePathProto* free_space_path_proto) {
  for (const GridVertex& gv : grid_path) {
    GridVertexToFreeSpaceProto(gv, free_space_path_proto->add_vertices());
  }
}

template <size_t kMaxRobotCount>
void PlannerStateToFreeSpacePathProto(
    const SearchWindow<kMaxRobotCount>& search_window,
    const PlannerState<kMaxRobotCount>& planner_state,
    MinuteBotsProto::EAstar::SearchWindowProto* search_window_proto) {
  const JointStepGridPath<kMaxRobotCount>& path = planner_state.repaired_paths;
  if (path.empty()) {
    return;
  }

  for (size_t i = 0; i < search_window.relevant_paths.size(); ++i) {
    GridPath individual_path;
    GridDistance prev_distance = -1;
    for (size_t position_index = 0; position_index < path.size();
         ++position_index) {
      const JointDistancePosition<kMaxRobotCount>& jdp =
          planner_state.repaired_paths.at(position_index);
      const GridVertex& v = jdp.position.at(i);
      const GridDistance& d = jdp.step.at(i).transit_steps;
      if (d != prev_distance) {
        if (prev_distance == -1 && d > 0) {
          for (int j = 0; j < d; ++j) {
            individual_path.push_back({kIntMaxHack, kIntMaxHack});
          }
        }
        individual_path.push_back(v);
        prev_distance = d;
      }
    }
    GridPathToFreeSpacePathProto(individual_path,
                                 search_window_proto->add_relevant_paths());
  }
}

template <size_t kMaxRobotCount>
void SearchWindowToSearchWindowProto(
    const SearchWindow<kMaxRobotCount>& search_window,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    MinuteBotsProto::EAstar::SearchWindowProto* search_window_proto) {
  WindowBoxToWindowBoxProto(search_window.box,
                            search_window_proto->mutable_box());
  for (const CollidingRobot& c : search_window.relevant_paths) {
    const GridPath& relevant_plan = slice.individual_plans.at(c.path_index);
    GridPathToFreeSpacePathProto(relevant_plan,
                                 search_window_proto->add_relevant_paths());
  }
}

template <size_t kMaxRobotCount>
void SearchWindowToSearchWindowProto(
    const SearchWindow<kMaxRobotCount>& search_window,
    const PlannerState<kMaxRobotCount>& planner_state,
    MinuteBotsProto::EAstar::SearchWindowProto* search_window_proto) {
  WindowBoxToWindowBoxProto(search_window.box,
                            search_window_proto->mutable_box());
  PlannerStateToFreeSpacePathProto(search_window, planner_state,
                                   search_window_proto);
}

template <size_t kMaxRobotCount>
void DumpCollisionWindows(
    const std::vector<SearchWindow<kMaxRobotCount>>& search_windows,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const size_t iteration_count) {
  static constexpr bool kDebug = false;
  MinuteBotsProto::EAstar::SearchWindowsWrapperProto
      search_windows_wrapper_proto;

  for (const SearchWindow<kMaxRobotCount>& search_window : search_windows) {
    SearchWindowToSearchWindowProto(
        search_window, slice,
        search_windows_wrapper_proto.add_search_windows());
  }

  const std::string file_name =
      "collision_windows_iter_" + std::to_string(iteration_count) + ".txt";
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);
  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(search_windows_wrapper_proto,
                                           &file_output)) {
    LOG(ERROR) << "Failed to write path proto to " << file_name;
  } else {
    if (kDebug) {
      LOG(INFO) << "Wrote file path proto "
                << ::util::serialization::GetFolderName() << file_name;
    }
  }
}

template <size_t kMaxRobotCount>
void DumpCollisionWindows(
    const std::vector<SearchWindow<kMaxRobotCount>>& search_windows,
    const std::vector<PlannerState<kMaxRobotCount>>& planner_states,
    const size_t iteration_count) {
  static constexpr bool kDebug = false;
  MinuteBotsProto::EAstar::SearchWindowsWrapperProto
      search_windows_wrapper_proto;

  for (size_t i = 0; i < search_windows.size(); ++i) {
    const SearchWindow<kMaxRobotCount>& search_window = search_windows.at(i);
    const PlannerState<kMaxRobotCount>& planner_state = planner_states.at(i);
    SearchWindowToSearchWindowProto(
        search_window, planner_state,
        search_windows_wrapper_proto.add_search_windows());
  }

  const std::string file_name =
      "collision_windows_iter_" + std::to_string(iteration_count) + ".txt";
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);
  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(search_windows_wrapper_proto,
                                           &file_output)) {
    LOG(ERROR) << "Failed to write path proto to " << file_name;
  } else {
    if (kDebug) {
      LOG(INFO) << "Wrote file path proto "
                << ::util::serialization::GetFolderName() << file_name;
    }
  }
}

template const std::vector<SearchWindow<kRoboCupEAStarMaxRobots>>
CollisionEventsToSearchWindows(
    const std::vector<CollisionEvent<kRoboCupEAStarMaxRobots>>&
        collision_events,
    const int search_window_radius);

template void DumpCollisionWindows(
    const std::vector<SearchWindow<kRoboCupEAStarMaxRobots>>& search_windows,
    const IndividuallyPlannedDataSlice<kRoboCupEAStarMaxRobots>& slice,
    const size_t iteration_count);

template void DumpCollisionWindows(
    const std::vector<SearchWindow<kRoboCupEAStarMaxRobots>>& search_windows,
    const std::vector<PlannerState<kRoboCupEAStarMaxRobots>>& planner_state,
    const size_t iteration_count);

}  // namespace eastar
}  // namespace search
