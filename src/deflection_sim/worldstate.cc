// Copyright 2016 - 2017 kvedder@umass.edu
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

#include "deflection_sim/worldstate.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::endl;
using std::vector;

namespace simulator {
WorldState::WorldState(vector<Robot> robots, Ball ball)
    : robots(robots), ball(ball) {}

WorldState::~WorldState() {}

void WorldState::Update(const RadioProtocolWrapper& command_wrapper) {
  ball.UpdatePosition();
  for (const RadioProtocolCommand& command : command_wrapper.command()) {
    if (command.robot_id() >= 0 &&
        command.robot_id() < static_cast<int>(robots.size())) {
      robots[command.robot_id()].Update(command, &ball);
    } else {
      LOG(ERROR) << "Velocity command given to non-existant robot of index "
                 << command.robot_id();
    }
  }
}

const Ball& WorldState::GetBall() const { return ball; }

const vector<Robot>& WorldState::GetRobots() const { return robots; }
}  // namespace simulator
