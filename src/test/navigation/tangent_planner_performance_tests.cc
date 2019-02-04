// Copyright 2017 - 2018 slane@umass.edu
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

#include <vector>

#include "eigen3/Eigen/Core"
#include "logging/logger.h"
#include "navigation/static_tangent_planner.h"
#include "test/navigation/scenario_generator.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

using Eigen::Vector2f;
using navigation::StaticTangentPlanner;
using std::pair;
using std::vector;

namespace navigation {
TEST(SmallSet, Example) {
  auto scenerio = navigation::test::GenerateSmallSet();
  StaticTangentPlanner planner(scenerio.obstacle_flag, scenerio.margin);

  logger::Logger log;

  planner.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
                 scenerio.goal, &log);
  planner.EnableDumpToFile("static_tangents.dat");
  pair<bool, std::vector<Eigen::Vector2f>> value = planner.Plan(&log);
}
}  // namespace navigation
