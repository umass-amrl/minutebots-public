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

#include "tactics/demo_tsocs.h"

#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>


#include <map>
#include <memory>

#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "tactics/tsocs_controller.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using state::WorldState;
using state::SharedState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;

extern ScopedFile tsocs_data_fid;

namespace tactics {

void DemoTSOCS::GenerateNewProblem() {
  if (trials_this_problem == trials_per_problem) {
    if (num_problems % 3 == 0) {
      start_pos = Eigen::Vector2f(2000, -1000);
      start_vel = Eigen::Vector2f(1000, 0);
      goal_pos = Eigen::Vector2f(2000, -2000);
      goal_vel = Eigen::Vector2f(1000, 1000);
    } else if (num_problems % 3 == 1) {
      start_pos = Eigen::Vector2f(2000, -1000);
      start_vel = Eigen::Vector2f(1000, 0);
      goal_pos = Eigen::Vector2f(3000, -2000);
      goal_vel = Eigen::Vector2f(0, -1000);
    } else {
      start_pos = Eigen::Vector2f(2000, -1000);
      start_vel = Eigen::Vector2f(1000, 0);
      goal_pos = Eigen::Vector2f(1000, -2000);
      goal_vel = Eigen::Vector2f(-1000, 1000);
    }
    trials_this_problem = 1;
    num_problems++;
  } else {
    trials_this_problem++;
  }

  if (kCollectTSOCSData) {
    fprintf(tsocs_data_fid,
      "GENERATED PROBLEM: %12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n",
       start_pos.x(), start_pos.y(), start_vel.x(), start_vel.y(),
       goal_pos.x(), goal_pos.y(), goal_vel.x(), goal_vel.y());
    fflush(tsocs_data_fid);
  }
}

}  // namespace tactics
