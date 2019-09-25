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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_EASTAR_SOLVER_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_EASTAR_SOLVER_H_

#include <vector>

#include "search/robocup_eastar/planner_state.h"
#include "search/robocup_eastar/search_window.h"

namespace search {
namespace eastar {

template <size_t kMaxRobotCount>
bool ShouldTerminateXStar(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    const std::vector<SearchWindow<kMaxRobotCount>>& search_windows,
    const std::vector<PlannerState<kMaxRobotCount>>& planner_states);

template <size_t kMaxRobotCount>
void RunExpandingAStar(
    const IndividuallyPlannedDataSlice<kMaxRobotCount>& slice,
    SearchWindow<kMaxRobotCount>* search_window,
    PlannerState<kMaxRobotCount>* planner_state, const float inflation);

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_EASTAR_SOLVER_H_
