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

#include "navigation/navigation.h"

namespace navigation {
Navigation::Navigation(const obstacle::ObstacleFlag& obstacles,
                       const obstacle::SafetyMargin& safety_margin,
                       const std::size_t rng_seed)
    : obstacles_(obstacles),
      safety_margin_(safety_margin),
      start_position_(0, 0),
      goal_position_(0, 0),
      random_(rng_seed) {}

Navigation::Navigation(const obstacle::ObstacleFlag& obstacles,
                       const obstacle::SafetyMargin& safety_margin)
    : obstacles_(obstacles),
      safety_margin_(safety_margin),
      start_position_(0, 0),
      goal_position_(0, 0) {}

void Navigation::Update(const obstacle::ObstacleFlag& obstacles,
                        const obstacle::SafetyMargin& safety_margin,
                        const Vector2f& current_pose, const Vector2f& goal_pose,
                        logger::Logger* logger) {
  obstacles_ = obstacles;
  safety_margin_ = safety_margin;
  start_position_ = current_pose;
  goal_position_ = goal_pose;
}

}  // namespace navigation
