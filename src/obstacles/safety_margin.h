// Copyright 2017-2018 kvedder@umass.edu, slane@cs.umass.edu
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

#ifndef SRC_OBSTACLES_SAFETY_MARGIN_H_
#define SRC_OBSTACLES_SAFETY_MARGIN_H_

#include "obstacles/obstacle_type.h"

namespace obstacle {
struct SafetyMargin {
 public:
  // Instantiates all margins to the default radius
  SafetyMargin();

  // Copy constructor
  SafetyMargin(const SafetyMargin& other);

  // Assignment operator
  void operator = (const SafetyMargin& other);

  // Sets the margin for the given obstacle type to the input value
  void SetMargin(const ObstacleType& type, float value);

  // Returns the margin for the given obstacle type
  float GetMargin(const ObstacleType& type) const;

  // Resets the margins to defaut for all obstacle types
  void ResetToDefault();

 private:
  float margins[ObstacleType::NUM_OBSTACLE_TYPES];
};
}  // namespace obstacle

#endif  // SRC_OBSTACLES_SAFETY_MARGIN_H_
