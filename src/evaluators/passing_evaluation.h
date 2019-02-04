// Copyright 2018 ikhatri@umass.edu
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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#ifndef SRC_EVALUATORS_PASSING_EVALUATION_H_
#define SRC_EVALUATORS_PASSING_EVALUATION_H_

#include <iostream>
#include <vector>
#include "constants/constants.h"
#include "evaluators/field_visualizer.h"
#include "evaluators/offense_evaluation.h"
#include "src/state/world_state.h"

namespace passing_evaluation {

float NormalizeDistance(float old_dist);
float NormalizeOpenAngle(float old_angle);
float PassAheadObjective(Eigen::Vector2f ball_pose,
                         std::vector<unsigned int> robots_to_ignore,
                         Eigen::Vector2f pass_ahead_pose,
                         const state::WorldState& world_state);

float GetPoseCost(Eigen::Vector2f ball,
                  std::vector<unsigned int> robots_to_ignore,
                  Eigen::Vector2f position,
                  const state::WorldState& world_state);

void save_pass_ahead_heatmap(Eigen::Vector2f ball_pose,
                             std::vector<unsigned int> robots_to_ignore,
                             Vector2f pass_ahead_pose,
                             const state::WorldState& world_state);

}  // namespace passing_evaluation

#endif  // SRC_EVALUATORS_PASSING_EVALUATION_H_
