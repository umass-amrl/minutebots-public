// Copyright 2017 slane@cs.umass.edu
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

#ifndef SRC_NAVIGATION_STATIC_TANGENT_PLANNER_H_
#define SRC_NAVIGATION_STATIC_TANGENT_PLANNER_H_

#include <utility>
#include <vector>
#include <string>

#include "algorithms/branch_and_bound.h"
#include "navigation/navigation.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

namespace navigation {

struct TangentAssignment {
  TangentAssignment();
  ~TangentAssignment() = default;

  float path_length;
  std::vector<unsigned int> obstacle_indices;
  std::vector<Eigen::Vector2f> path;
  std::vector<bool> is_right;

  bool operator<(const TangentAssignment& other) const {
    return path_length < other.path_length;
  }

  bool operator==(const TangentAssignment& other) const {
    return path_length == other.path_length;
  }

  bool operator>(const TangentAssignment& other) const {
    return path_length > other.path_length;
  }

  // TODO(slane) figure out a better way to do this than static class members
  static Eigen::Vector2f start;
  static Eigen::Vector2f goal;
};

class StaticTangentPlanner :
    public Navigation, public algorithms::BranchAndBound<float,
                                                         float,
                                                         TangentAssignment>{
 public:
  StaticTangentPlanner(obstacle::ObstacleFlag obstacle_flag,
                       obstacle::SafetyMargin margin);

  std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* logger) override;

  void EnableDumpToFile(std::string filename);

 private:
  float CalculateHeuristic(const TangentAssignment& assignment) override;
  float CalculateLowerBound(const TangentAssignment& assignment) override;
  float CalculateUpperBound(const TangentAssignment& assignment) override;
  bool Optimize(const TangentAssignment& assignment,
                float* optimal_cost) override;
  void Branch(const TangentAssignment& assignment,
              std::vector<TangentAssignment>* branches) override;

  bool IsInAssignment(const TangentAssignment& assignment, unsigned int index);

  float GetPathLength(const TangentAssignment& assignment);

  void GetBitangentPoints(const Vector2f& p1,
                          const Vector2f& p2,
                          const float& r,
                          bool from_right,
                          Vector2f* right_tangenta,
                          Vector2f* right_tangentb,
                          Vector2f* left_tangenta,
                          Vector2f* left_tangentb);

  void DumpAssignment(const TangentAssignment& assignment);

  std::vector<Eigen::Vector2f> robot_centers;
  bool dump_to_file_;
  string filename_;
};
}  // namespace navigation

#endif  // SRC_NAVIGATION_STATIC_TANGENT_PLANNER_H_
