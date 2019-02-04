// Copyright 2017 - 2018 slane@cs.umass.edu, kvedder@umass.edu
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

#include "obstacles/safety_margin.h"

#include "constants/constants.h"
#include "obstacles/obstacle_type.h"

namespace obstacle {

SafetyMargin::SafetyMargin() {
  for (int i = 0; i < ObstacleType::NUM_OBSTACLE_TYPES; i++) {
    margins[i] = kDefaultSafetyMargin;
  }

  margins[ObstacleType::STATIC] = kDefaultRulesSafetyMargin;
  margins[ObstacleType::RULE] = kDefaultRulesSafetyMargin;
  margins[ObstacleType::BALL] = kRotationRadius - kEpsilon;
}

SafetyMargin::SafetyMargin(const SafetyMargin& other) {
  for (int i = 0; i < ObstacleType::NUM_OBSTACLE_TYPES; i++) {
    this->margins[i] = other.margins[i];
  }
}

void SafetyMargin::operator=(const SafetyMargin& other) {
  if (this != &other) {
    for (int i = 0; i < ObstacleType::NUM_OBSTACLE_TYPES; i++) {
      this->margins[i] = other.margins[i];
    }
  }
}

void SafetyMargin::SetMargin(const ObstacleType& type, float value) {
  margins[type] = value;
}

float SafetyMargin::GetMargin(const ObstacleType& type) const {
  return margins[type];
}

void SafetyMargin::ResetToDefault() {
  for (int i = 0; i < ObstacleType::NUM_OBSTACLE_TYPES; i++) {
    margins[i] = kDefaultSafetyMargin;
  }
}


}  // namespace obstacle
