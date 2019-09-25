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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_SEARCH_WINDOW_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_SEARCH_WINDOW_H_

#include <algorithm>
#include <vector>

#include "search/robocup_eastar/multiagent_data.h"
#include "search/robocup_eastar/path_collision_detector.h"

namespace search {
namespace eastar {

struct WindowBox {
  GridVertex upper_left;
  GridVertex lower_right;
  int expansion_delta;

  WindowBox() : upper_left(0, 0), lower_right(0, 0), expansion_delta(0) {}
  WindowBox(const GridVertex& upper_right, const GridVertex& lower_left,
            const int& expansion_delta)
      : upper_left(upper_right),
        lower_right(lower_left),
        expansion_delta(expansion_delta) {
    NP_CHECK_MSG(upper_left.x() <= lower_right.x(),
                 "UL: (" << upper_left.x() << ", " << upper_left.y()
                         << ") LL: (" << lower_right.x() << ", "
                         << lower_right.y() << ")");
    NP_CHECK_MSG(upper_left.y() >= lower_right.y(),
                 "UL: (" << upper_left.x() << ", " << upper_left.y()
                         << ") LL: (" << lower_right.x() << ", "
                         << lower_right.y() << ")");
  }

  void Expand() {
    upper_left += GridVertex(-expansion_delta, expansion_delta);
    lower_right -= GridVertex(-expansion_delta, expansion_delta);
  }

  bool Inside(const GridVertex& p) const { return InXRange(p) && InYRange(p); }

  bool Inside(const FreeSpaceVertex& p) const {
    return InXRange(p) && InYRange(p);
  }

  bool Intersects(const WindowBox& other) const {
    if (Inside(other.lower_right)) {
      return true;
    }
    if (Inside(other.upper_left)) {
      return true;
    }
    const GridVertex other_upper_right = other.ComputeUpperRight();
    if (Inside(other_upper_right)) {
      return true;
    }
    const GridVertex other_lower_left = other.ComputeLowerLeft();
    if (Inside(other_lower_left)) {
      return true;
    }
    return false;
  }

  WindowBox Merge(const WindowBox& other) const {
    NP_CHECK(other.expansion_delta == expansion_delta);
    const GridVertex new_upper_left(
        std::min(upper_left.x(), other.upper_left.x()),
        std::max(upper_left.y(), other.upper_left.y()));
    const GridVertex new_lower_right(
        std::max(lower_right.x(), other.lower_right.x()),
        std::min(lower_right.y(), other.lower_right.y()));
    return {new_upper_left, new_lower_right, expansion_delta};
  }

 private:
  bool InXRange(const GridVertex& p) const {
    return (p.x() <= lower_right.x() && p.x() >= upper_left.x());
  }
  bool InYRange(const GridVertex& p) const {
    return (p.y() >= lower_right.y() && p.y() <= upper_left.y());
  }

  bool InXRange(const FreeSpaceVertex& p) const {
    return (p.x() * kEightGridSquareSize <= lower_right.x() &&
            p.x() * kEightGridSquareSize >= upper_left.x());
  }
  bool InYRange(const FreeSpaceVertex& p) const {
    return (p.y() * kEightGridSquareSize >= lower_right.y() &&
            p.y() * kEightGridSquareSize <= upper_left.y());
  }

  GridVertex ComputeUpperRight() const {
    return {lower_right.x(), upper_left.y()};
  }

  GridVertex ComputeLowerLeft() const {
    return {upper_left.x(), lower_right.y()};
  }
};

template <size_t kMaxRobotCount>
struct SearchWindow {
  WindowBox box;
  CollidingRobotArray<kMaxRobotCount> relevant_paths;
  SearchWindow() = default;
  SearchWindow(const WindowBox& box,
               const CollidingRobotArray<kMaxRobotCount>& relevant_paths)
      : box(box), relevant_paths(relevant_paths) {}

  bool Intersects(const SearchWindow& other) const {
    return box.Intersects(other.box);
  }

  SearchWindow Merge(const SearchWindow& other) const {
    const WindowBox new_box = box.Merge(other.box);
    SearchWindow new_window(new_box, relevant_paths);

    // Add additional paths if they are not duplicates.
    for (const CollidingRobot& candidate : other.relevant_paths) {
      bool is_duplicate = false;
      for (const CollidingRobot& existing : new_window.relevant_paths) {
        if (existing.path_index == candidate.path_index) {
          is_duplicate = true;
          break;
        }
      }
      if (!is_duplicate) {
        new_window.relevant_paths.push_back(candidate);
      }
    }
    return new_window;
  }

  void AddNewPaths(
      const datastructures::DenseArray<OurRobotIndex, kMaxRobotCount>& paths) {
    for (const OurRobotIndex& i : paths) {
      NP_CHECK(!relevant_paths.Contains(CollidingRobot(i)));
      relevant_paths.push_back(CollidingRobot(i));
    }
  }

  void Expand() { box.Expand(); }
};

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const SearchWindow<kMaxRobotCount>& event) {
  os << "Box: UL: (" << event.box.upper_left.x() << ", "
     << event.box.upper_left.y() << ") LL: (" << event.box.lower_right.x()
     << ", " << event.box.lower_right.y() << ")\n Relevant paths:\n";
  for (const auto& r : event.relevant_paths) {
    os << "\tPath index: " << r.path_index << '\n';
  }
  return os;
}

template <size_t kMaxRobotCount>
const std::vector<SearchWindow<kMaxRobotCount>> CollisionEventsToSearchWindows(
    const std::vector<CollisionEvent<kMaxRobotCount>>& collision_events,
    const int search_window_radius);

template <size_t kMaxRobotCount>
void DumpCollisionWindows(
    const std::vector<SearchWindow<kMaxRobotCount>>& search_windows,
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const size_t iteration_count);

template <size_t kMaxRobotCount>
struct PlannerState;

template <size_t kMaxRobotCount>
void DumpCollisionWindows(
    const std::vector<SearchWindow<kMaxRobotCount>>& search_windows,
    const std::vector<PlannerState<kMaxRobotCount>>& planner_states,
    const size_t iteration_count);

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_SEARCH_WINDOW_H_
