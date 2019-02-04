// Copyright 2017 slane@cs.umass.edu, jaholtz@cs.umass.edu
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

#ifndef SRC_EVALUATORS_STOPPED_EVALUATION_H_
#define SRC_EVALUATORS_STOPPED_EVALUATION_H_

#include <vector>

#include "evaluators/offense_evaluation.h"
#include "state/soccer_state.h"
#include "state/world_state.h"

namespace defense {
class StoppedEvaluator {
 public:
  StoppedEvaluator();

  void SetStoppedTargets(const state::WorldState& world_state,
                         const state::SoccerState& soccer_state);

  void AssignPoses(
    const std::vector<OurRobotIndex>& robot_indices,
    const state::WorldState& world_state,
    const state::SoccerState& soccer_state,
    std::vector<pose_2d::Pose2Df>* poses_to_assign);
};
}  // namespace defense

#endif  // SRC_EVALUATORS_STOPPED_EVALUATION_H_
