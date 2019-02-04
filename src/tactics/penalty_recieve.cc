// Copyright 2017 - 2018 slane@cs.umass.edu
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

#include "tactics/penalty_recieve.h"

#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/defense_evaluation.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "radio_protocol_wrapper.pb.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"

#define _USE_MATH_DEFINES

STANDARD_USINGS;
using defense::DefenseEvaluator;
using Eigen::Vector2f;
using pose_2d::Pose2Df;
using state::WorldState;
using state::SharedState;
using std::atan2;
using std::endl;
using std::map;
using std::tan;
using std::unique_ptr;
using tactics::TacticIndex;

namespace tactics {

PenaltyRecieve::PenaltyRecieve(const WorldState& world_state,
                               TacticArray* tactic_list,
                               SharedState* shared_state,
                               OurRobotIndex our_robot_index,
                               state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state),
      guard_pose_(0, 0, 0) {}

void PenaltyRecieve::Init() {}

void PenaltyRecieve::Run() {
  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  controller->SetGoal(guard_pose_);
  controller->Run();
}

void PenaltyRecieve::Reset() { guard_pose_ = {0, 0, 0}; }

void PenaltyRecieve::SetGoal(const Pose2Df& pose) { guard_pose_ = pose; }

}  // namespace tactics
