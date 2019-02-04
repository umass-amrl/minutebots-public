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

#ifndef SRC_MOTION_CONTROL_ANGULAR_PLANNER_H_
#define SRC_MOTION_CONTROL_ANGULAR_PLANNER_H_

#include "logging/logger.h"
#include "motion_control/motion_model.h"

namespace motion {
float CalculateAngularWaypoint(float goal_angle,
                               float current_angle,
                               double translation_arrival_time,
                               double pre_arrival_delta,
                               double control_period,
                               float max_velocity,
                               logger::Logger* logger);
}  // namespace motion

#endif  // SRC_MOTION_CONTROL_ANGULAR_PLANNER_H_
