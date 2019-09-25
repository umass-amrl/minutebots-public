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

#ifndef SRC_SEARCH_FOURGRID_FOURGRID_SCENERIO_H_
#define SRC_SEARCH_FOURGRID_FOURGRID_SCENERIO_H_

#include <vector>

#include "constants/constants.h"
#include "search/fourgrid/fourgrid.h"

namespace search {
namespace fourgrid {
namespace demo {

template <size_t kRobotCount>
struct FourgridScenerio {
  std::vector<::search::fourgrid::FourGrid<kRobotCount>> four_grids;
  std::vector<::search::fourgrid::CostPath<kRobotCount>> start_connection_paths;
  std::vector<::search::fourgrid::JointPosition<kRobotCount>> starts;
  std::vector<::search::fourgrid::JointPosition<kRobotCount>> goals;
  ::search::fourgrid::FourGrid<kRobotCount> fourgrid_individual;

  FourgridScenerio() = delete;
  FourgridScenerio(
      const std::vector<::search::fourgrid::FourGrid<kRobotCount>>& four_grids,
      const std::vector<::search::fourgrid::CostPath<kRobotCount>>&
          start_connection_paths,
      const std::vector<::search::fourgrid::JointPosition<kRobotCount>>& starts,
      const std::vector<::search::fourgrid::JointPosition<kRobotCount>>& goals,
      const ::search::fourgrid::FourGrid<kRobotCount>& fourgrid_individual)
      : four_grids(four_grids),
        start_connection_paths(start_connection_paths),
        starts(starts),
        goals(goals),
        fourgrid_individual(fourgrid_individual){}

  ::search::fourgrid::FourGrid<kRobotCount> GetLargest() const {
    return four_grids[four_grids.size() - 1];
  }

  void Verify() const {
    NP_CHECK(!four_grids.empty());
    NP_CHECK_EQ(four_grids.size(), start_connection_paths.size());
    NP_CHECK_EQ(four_grids.size(), starts.size());
    NP_CHECK_EQ(four_grids.size(), goals.size());
    NP_CHECK_EQ(fourgrid_individual.GetUniquePositions().size(),
                GetLargest().GetUniquePositions().size());
  }
};

}  // namespace demo
}  // namespace fourgrid
}  // namespace search

#endif  // SRC_SEARCH_FOURGRID_FOURGRID_SCENERIO_H_
