// Copyright 2016 - 2019 jaholtz@cs.umass.edu
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

#include "experimental_sim/sim_state.h"
#include <set>
#include "experimental_sim/objects/ball.h"
#include "experimental_sim/objects/robot.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::endl;
using std::vector;

namespace experimental_simulator {
// Sets up robots in initial positions.
// TODO(kvedder): Decide upon a better means of initializing robots.
vector<Robot> CreateRobots(const SimState& world_state, const int num_robots,
                           const float time_slice) {
  vector<Robot> robots;
  for (int i = 0; i < num_robots; ++i) {
    const Pose2Df pose(0.0, Vector2f(500 * i - 3000, 500));
    const Pose2Df velocity(0.0, Vector2f(0.0, 0.0));
    const int id = i;
    const team::Team team = ((static_cast<size_t>(i) < kMaxTeamRobots) ?
        team::Team::BLUE :
        team::Team::YELLOW);
    robots.push_back(Robot(world_state, pose, velocity, id, team, time_slice));
  }
  return robots;
}

vector<Robot> CreateRobots(const SimState& world_state,
                           const vector<Pose2Df>& positions,
                           const vector<Pose2Df>& velocities,
                           const float time_slice) {
  vector<Robot> robots;
  for (size_t i = 0; i < positions.size(); ++i) {
    const int id = i;
    const team::Team team = ((i < kMaxTeamRobots) ?
        team::Team::BLUE :
        team::Team::YELLOW);
    robots.push_back(Robot(world_state,
                           positions[i],
                           velocities[i],
                           id,
                           team,
                           time_slice));
  }
  return robots;
}

vector<Robot> CreateRobots(const SimState& world_state,
                           const vector<Pose2Df>& positions,
                           const float time_slice) {
  vector<Robot> robots;
  for (size_t i = 0; i < positions.size(); ++i) {
    const Pose2Df velocity(0.0, Vector2f(0.0, 0.0));
    const int id = i;
    const team::Team team = ((i < kMaxTeamRobots) ?
    team::Team::BLUE :
    team::Team::YELLOW);
    robots.push_back(Robot(world_state,
                           positions[i],
                           velocity,
                           id,
                           team,
                           time_slice));
  }
  return robots;
}

const float& SimState::GetFriction() const {
  return kFrictionCoefficient_;
}

void SimState::SetBallPose(const Vector2f& pose) {
  ball.SetPose(pose);
}

void SimState::SetBallVelocity(const Vector2f& velocity) {
  ball.SetVelocity(velocity);
}

Ball CreateBall(const SimState& world_state, const float time_slice) {
  return Ball(world_state, Pose2Df(0, Vector2f(0, 0)), Vector2f(0, 0),
              time_slice);
}

SimState::SimState(const int& num_robots,
                   const float& friction_coefficient,
                   const float& time_slice)
    : robots(CreateRobots(*this, num_robots, time_slice)),
      ball(CreateBall(*this, time_slice)),
      kFrictionCoefficient_(friction_coefficient) {}

SimState::SimState(const vector<Pose2Df>& positions,
                   const vector<Pose2Df>& velocities,
                   const float& friction_coefficient,
                   const float& time_slice)
    : robots(CreateRobots(*this, positions, velocities, time_slice)),
      ball(CreateBall(*this, time_slice)),
      kFrictionCoefficient_(friction_coefficient) {}

void SimState::Update(const RadioProtocolWrapper& command_wrapper,
                      const float& step_time) {
  std::set<int> processed_ids;
  for (const RadioProtocolCommand& command : command_wrapper.command()) {
    if (command.robot_id() >= 0 &&
        command.robot_id() < static_cast<int>(robots.size())) {
      robots[command.robot_id()].Update(command, step_time, &ball);
    processed_ids.insert(command.robot_id());
    } else {
      LOG(ERROR) << "Velocity command given to non-existant robot of index "
                 << command.robot_id();
    }
  }
  for (Robot& robot : robots) {
    if (!(processed_ids.count(robot.GetId()) > 0)) {
      robot.Update(&ball, step_time);
    }
  }
  ball.UpdatePosition();
}

void SimState::SetBallAccel(const float& ball_accel) {
  ball.ball_acceleration_ = ball_accel;
}


const Ball& SimState::GetBall() const { return ball; }

void SimState::SetBall(Ball new_ball) {
  ball.SetPose(new_ball.GetPose2Df().translation);
  ball.SetVelocity(new_ball.GetVelocity());
}

const vector<Robot>& SimState::GetRobots() const { return robots; }
}  // namespace experimental_simulator
