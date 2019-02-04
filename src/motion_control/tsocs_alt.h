// Copyright 2017 dbalaban@cs.umass.edu
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

#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/types.h>
#include <math.h>

#include <cmath>
#include <algorithm>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "shared/common_includes.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/sub_optimal_controls.h"
#include "motion_control/motion_control_structures.h"
#include "logging/logger.h"

#ifndef SRC_MOTION_CONTROL_TSOCS_ALT_H_
#define SRC_MOTION_CONTROL_TSOCS_ALT_H_

namespace temp {

// finds a set of parameters which were used to find optimal solution
// given starting / ending position and velocity
// bounded acceleration, unbounded velocity
// returns true if correct solution was found
bool GetSolutionSet_alt(Eigen::Vector2d x0, Eigen::Vector2d v0,
                 Eigen::Vector2d xf, Eigen::Vector2d vf,
                 const double a_max,
                 std::vector<SolutionParameters_alt>* params);

// finds a single parameter set
// returns true if correct solution was found
bool GetSolution_alt(Eigen::Vector2d x0, Eigen::Vector2d v0,
                    Eigen::Vector2d xf, Eigen::Vector2d vf,
                    const double a_max,
                    SolutionParameters_alt* params);

bool GetSolution_alt(Eigen::Vector2d x0, Eigen::Vector2d v0,
                    Eigen::Vector2d xf, Eigen::Vector2d vf,
                    const double a_max,
                    SolutionParameters_alt* params,
                    logger::Logger* the_log);

// finds the expected position and velocity after time t
// given starting position and velocity and an optimal solution
// bounded acceleration, unbounded velocity
void GetState_alt(Eigen::Vector2d x0, Eigen::Vector2d v0,
              Eigen::Vector2d* xt, Eigen::Vector2d* vt,
              const double a_max, const double t,
              SolutionParameters_alt params);

void GetState_alt(Eigen::Vector2d x0, Eigen::Vector2d v0,
              Eigen::Vector2d* xt, Eigen::Vector2d* vt,
              const double a_max, const double t,
              SolutionParameters_alt params, logger::Logger* the_log);

std::vector<Eigen::Vector2f> GetPath_alt(Eigen::Vector2d robot_pos_init,
                                     Eigen::Vector2d robot_vel_init,
                                     const double a_max,
                                     const unsigned int npts,
                                     SolutionParameters_alt params);

}  // namespace temp
#endif  // SRC_MOTION_CONTROL_TSOCS_ALT_H_
