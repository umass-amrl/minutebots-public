// Copyright 2017 - 2018 rezecib@gmail.com
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

#include "tactics/guard_point.h"

#include <memory>
#include <string>
#include <vector>

#include "constants/constants.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/coordinates.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using state::WorldState;
using state::SharedState;
using state::WorldRobot;
using std::unique_ptr;
using std::stof;
using tactics::TacticIndex;
using geometry::Angle;
using coordinates::ParseCoordinates;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;

namespace tactics {

GuardPoint::GuardPoint(const WorldState& world_state,
                       TacticArray* tactic_list,
                       SharedState* shared_state, OurRobotIndex our_robot_index,
                       state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

void GuardPoint::Init() {
  point_ = Vector2f(0, 0);
  leash_ = 200;
}

void GuardPoint::Run() {
  Pose2Df robot_goal_pose;

  robot_goal_pose = {0, point_};

  Tactic* controller = (*tactic_list_)[TacticIndex::EIGHT_GRID].get();
  controller->SetGoal(robot_goal_pose);
  controller->Run();
}

void GuardPoint::Reset() {
  point_ = Vector2f(0, 0);
  leash_ = 200;
}

void GuardPoint::SetGoal(const Pose2Df& pose) {}

void GuardPoint::SetParameters(const vector<string>& parameters) {
  point_ = ParseCoordinates(world_state_, parameters.at(1));
  leash_ = stof(parameters.at(2));
}

float GuardPoint::GetCost() {
  // Return the distance from the target guard point
  // This could be improved by using time-to-fulfill
  auto robot = world_state_.GetOurRobotPosition(our_robot_index_);

  MotionModel motion_model(kMaxRobotAcceleration, kMaxRobotVelocity);
  ControlSequence2D linear_control;
  Vector2f ntoc_position;
  Vector2f ntoc_velocity;
  ntoc::TransformCoordinatesForNTOC(robot.position, robot.velocity.translation,
                                    point_, &ntoc_position, &ntoc_velocity);

  ntoc_velocity.x() = 0;
  ntoc_velocity.y() = 0;

  return (NTOC2D(ntoc_position, ntoc_velocity, motion_model, &linear_control));
}

}  // namespace tactics
