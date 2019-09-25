// Copyright 2018 kvedder@umass.edu
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

#ifndef SRC_NAVIGATION_REPAIR_EIGHT_GRID_DISTANCE_H_
#define SRC_NAVIGATION_REPAIR_EIGHT_GRID_DISTANCE_H_

#include <limits>
#include <string>

#include "src/constants/constants.h"

namespace navigation {
namespace repair {
namespace repairer {

struct Distance {
  int straight_count = 0;
  int angle_count = 0;

  Distance()
      : straight_count(std::numeric_limits<int>::max()),
        angle_count(std::numeric_limits<int>::max()) {}
  Distance(const int& straight_count, const int& angle_count)
      : straight_count(straight_count), angle_count(angle_count) {
    if (!kProduction && IsNegative()) {
      LOG(FATAL) << "as constructed IsNegative! " << ToString();
    }
  }
  Distance(const Distance& other) = default;
  Distance& operator=(const Distance& other) = default;

  bool operator==(const Distance& other) const {
    if (!kProduction && IsNegative()) {
      LOG(FATAL) << "this IsNegative! " << ToString();
    }
    if (!kProduction && other.IsNegative()) {
      LOG(FATAL) << "other IsNegative! " << other.ToString();
    }
    if (!kProduction && ComponentsNegative()) {
      LOG(FATAL) << "this ComponentsNegative! " << ToString();
    }
    if (!kProduction && other.ComponentsNegative()) {
      LOG(FATAL) << "other ComponentsNegative! " << other.ToString();
    }
    return (straight_count == other.straight_count) &&
           (angle_count == other.angle_count);
  }

  bool operator!=(const Distance& other) const { return !(*this == other); }

  Distance operator+(const Distance& other) const {
    if (!kProduction && IsNegative()) {
      LOG(FATAL) << "this IsNegative! " << ToString();
    }
    if (!kProduction && other.IsNegative()) {
      LOG(FATAL) << "other IsNegative! " << other.ToString();
    }
    if (!kProduction && ComponentsNegative()) {
      LOG(FATAL) << "this ComponentsNegative! " << ToString();
    }
    if (!kProduction && other.ComponentsNegative()) {
      LOG(FATAL) << "other ComponentsNegative! " << other.ToString();
    }
    Distance result = {straight_count + other.straight_count,
                       angle_count + other.angle_count};
    if (!kProduction && result.IsNegative()) {
      LOG(FATAL) << "result IsNegative! " << result.ToString();
    }
    if (!kProduction && result.ComponentsNegative()) {
      LOG(FATAL) << "result ComponentsNegative! " << result.ToString();
    }
    return result;
  }

  Distance operator-(const Distance& other) const {
    if (!kProduction && IsNegative()) {
      LOG(FATAL) << "this IsNegative! " << ToString();
    }
    if (!kProduction && other.IsNegative()) {
      LOG(FATAL) << "other IsNegative! " << other.ToString();
    }
    if (!kProduction && ComponentsNegative()) {
      LOG(FATAL) << "this ComponentsNegative! " << ToString();
    }
    if (!kProduction && other.ComponentsNegative()) {
      LOG(FATAL) << "other ComponentsNegative! " << other.ToString();
    }
    Distance result = {straight_count - other.straight_count,
                       angle_count - other.angle_count};
    if (!kProduction && result.IsNegative()) {
      LOG(FATAL) << "result IsNegative! " << result.ToString();
    }
    if (!kProduction && result.ComponentsNegative()) {
      LOG(FATAL) << "result ComponentsNegative! " << result.ToString();
    }
    return result;
  }

  void operator+=(const Distance& other) {
    if (!kProduction && IsNegative()) {
      LOG(FATAL) << "this IsNegative! " << ToString();
    }
    if (!kProduction && other.IsNegative()) {
      LOG(FATAL) << "other IsNegative! " << other.ToString();
    }
    if (!kProduction && ComponentsNegative()) {
      LOG(FATAL) << "this ComponentsNegative! " << ToString();
    }
    if (!kProduction && other.ComponentsNegative()) {
      LOG(FATAL) << "other ComponentsNegative! " << other.ToString();
    }
    straight_count += other.straight_count;
    angle_count += other.angle_count;
  }

  bool operator<(const Distance& other) const {
    return (this->ToFloat() < other.ToFloat());
  }

  bool operator<=(const Distance& other) const {
    return (this->ToFloat() <= other.ToFloat());
  }

  bool operator>(const Distance& other) const {
    return (this->ToFloat() > other.ToFloat());
  }

  bool operator>=(const Distance& other) const {
    return (this->ToFloat() >= other.ToFloat());
  }

  float ToFloat() const { return (straight_count + kSqrtTwo * angle_count); }

  float ToFloat(const float& scalar) const {
    return scalar * (straight_count + kSqrtTwo * angle_count);
  }

  float ToTime(const float& scalar, const float& max_velocity) const {
    return ToFloat(scalar) / max_velocity;
  }

  Distance AverageWith(const Distance& other) const {
    if (!kProduction && IsNegative()) {
      LOG(FATAL) << "IsNegative! " << ToString();
    }
    if (!kProduction && other.IsNegative()) {
      LOG(FATAL) << "other IsNegative! " << other.ToString();
    }
    if (!kProduction && ComponentsNegative()) {
      LOG(FATAL) << "this ComponentsNegative! " << ToString();
    }
    if (!kProduction && other.ComponentsNegative()) {
      LOG(FATAL) << "other ComponentsNegative! " << other.ToString();
    }
    return {(this->straight_count + other.straight_count) / 2,
            (this->angle_count + other.angle_count) / 2};
  }

  std::string ToString() const {
    return "Distance: (" + std::to_string(straight_count) + ", " +
           std::to_string(angle_count) + ")";
  }

  bool IsNegative() const { return ToFloat() < 0.0f; }
  bool ComponentsNegative() const {
    return (straight_count < 0) || (angle_count < 0);
  }
};

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_EIGHT_GRID_DISTANCE_H_
