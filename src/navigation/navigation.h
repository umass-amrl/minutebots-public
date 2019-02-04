// Copyright 2017 kvedder@umass.edu
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

#ifndef SRC_NAVIGATION_NAVIGATION_H_
#define SRC_NAVIGATION_NAVIGATION_H_

#include <utility>
#include <vector>

#include "constants/constants.h"
#include "logging/logger.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"
#include "util/random.h"

namespace navigation {
class Navigation {
 public:
  Navigation() = default;
  Navigation(const obstacle::ObstacleFlag& obstacles,
             const obstacle::SafetyMargin& safety_margin);
  Navigation(const obstacle::ObstacleFlag& obstacles,
             const obstacle::SafetyMargin& safety_margin,
             const size_t rng_seed);
  virtual ~Navigation() = default;

  virtual void Update(const obstacle::ObstacleFlag& obstacles,
                      const obstacle::SafetyMargin& safety_margin,
                      const Eigen::Vector2f& current_pose,
                      const Eigen::Vector2f& goal_pose, logger::Logger* logger);

  virtual std::pair<bool, std::vector<Eigen::Vector2f>> Plan(
      logger::Logger* logger) = 0;

 protected:
  obstacle::ObstacleFlag obstacles_;
  obstacle::SafetyMargin safety_margin_;
  Eigen::Vector2f start_position_;
  Eigen::Vector2f goal_position_;
  util_random::Random random_;
};
}  // namespace navigation

#endif  // SRC_NAVIGATION_NAVIGATION_H_
