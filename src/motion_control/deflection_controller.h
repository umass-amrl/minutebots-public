// Copyright 2017 dbalaban@cs.umass.edu
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

#ifndef SRC_MOTION_CONTROL_DEFLECTION_CONTROLLER_H_
#define SRC_MOTION_CONTROL_DEFLECTION_CONTROLLER_H_

#include <cmath>
#include <vector>

#include "math/poses_2d.h"
#include "math/math_util.h"
#include "shared/common_includes.h"
#include "motion_control/optimal_control_1d.h"
#include "eigen3/Eigen/Dense"

using Eigen::Vector2d;
using pose_2d::Pose2Dd;
using math_util::SolveQuadratic;

// gets the necessary velocity for the next time step
pose_2d::Pose2Dd DeflectControl(const pose_2d::Pose2Dd& robot_pose,
                              const Eigen::Vector2d& ball_pose,
                              const pose_2d::Pose2Dd& robot_velocity,
                              const Eigen::Vector2d& ball_velocity,
                              const Eigen::Vector2d& goal);

#endif  // SRC_MOTION_CONTROL_DEFLECTION_CONTROLLER_H_
