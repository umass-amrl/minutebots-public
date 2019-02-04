// Copyright 2016 - 2018 kvedder@umass.edu
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

#include "sim/worldstate.h"
#include "sim/objects/ball.h"
#include "sim/objects/robot.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::endl;
using std::vector;

namespace simulator {
// Sets up robots in initial positions.
// TODO(kvedder): Decide upon a better means of initializing robots.
vector<Robot> CreateRobots(const WorldState& world_state, const int num_robots,
                           const float time_slice) {
  vector<Robot> robots;
  for (int i = 0; i < num_robots; ++i) {
    const Pose2Df pose(0.0, Vector2f(500 * i - 3000, 500));
    const Vector2f velocity(0, 0);
    const int id = i;
    const team::Team team = ((i < 6) ? team::Team::BLUE : team::Team::YELLOW);
    robots.push_back(Robot(world_state, pose, velocity, id, team, time_slice));
  }
  return robots;
}

vector<Robot> CreateRobots(const WorldState& world_state,
                           const vector<Vector2f>& positions,
                           const float time_slice) {
  vector<Robot> robots;
  for (size_t i = 0; i < positions.size(); ++i) {
    const Pose2Df pose(0.0, positions[i]);
    const Vector2f velocity(0, 0);
    const int id = i;
    const team::Team team = ((i < 6) ? team::Team::BLUE : team::Team::YELLOW);
    robots.push_back(Robot(world_state, pose, velocity, id, team, time_slice));
  }
  return robots;
}

Ball CreateBall(const WorldState& world_state, const float time_slice) {
  const float ball_mass = 0.08f;
  const float friction_coefficient = 0.0005f;
  return Ball(world_state, Pose2Df(0, Vector2f(0, 0)), Vector2f(0, 0),
              ball_mass, time_slice, friction_coefficient);
}

WorldState::WorldState(const int num_robots, const float time_slice)
    : robots(CreateRobots(*this, num_robots, time_slice)),
      ball(CreateBall(*this, time_slice)) {}

WorldState::WorldState(const vector<Vector2f>& positions,
                       const float time_slice)
    : robots(CreateRobots(*this, positions, time_slice)),
      ball(CreateBall(*this, time_slice)) {}

void WorldState::Update(const RadioProtocolWrapper& command_wrapper) {
  for (const RadioProtocolCommand& command : command_wrapper.command()) {
    if (command.robot_id() >= 0 &&
        command.robot_id() < static_cast<int>(robots.size())) {
      robots[command.robot_id()].Update(command, &ball);
    } else {
      LOG(ERROR) << "Velocity command given to non-existant robot of index "
                 << command.robot_id();
    }
  }
  ball.UpdatePosition();
}

const Ball& WorldState::GetBall() const { return ball; }

const vector<Robot>& WorldState::GetRobots() const { return robots; }
}  // namespace simulator
