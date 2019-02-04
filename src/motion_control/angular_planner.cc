// Copyright 2018 slane@cs.umass.edu
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


#include "motion_control/angular_planner.h"

#include <algorithm>

#include "constants/constants.h"
#include "logging/logger.h"
#include "math/math_util.h"

STANDARD_USINGS;

using math_util::AngleDiff;
using logger::Logger;

namespace motion {

float CalculateAngularWaypoint(float goal_angle,
                               float current_angle,
                               double translation_arrival_time,
                               double pre_arrival_delta,
                               double control_period,
                               float max_velocity,
                               Logger* logger) {
  float delta_angle = AngleDiff(goal_angle, current_angle);

  if (translation_arrival_time - pre_arrival_delta <= 0) {
    return delta_angle;
  } else {
    float desired_speed = delta_angle/
                      static_cast<float>(translation_arrival_time -
                                          pre_arrival_delta);
    desired_speed = min(desired_speed, max_velocity);
    return desired_speed*control_period;
  }
}


}  // namespace motion
