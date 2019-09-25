// Copyright 2017 - 2019 jaholtz@cs.umass.edu
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

#include "experimental_sim/experimental_sim.h"
#include "constants/constants.h"
#include "logging/logger.h"
#include "state/soccer_state.h"
#include "state/world_state.h"

using experimental_simulator::SimState;
using experimental_simulator::ExperimentalSim;
using state::PositionVelocityState;

const double noise_level = 0.00;
std::default_random_engine generator;
std::normal_distribution<double> distribution(1.0, noise_level);

int main() {
  const double time_step = 1.0f / 60.0f;
  const int kRobots = 1;
  SimState state(kRobots, 0.0005f, time_step);
  ExperimentalSim simulator(time_step, &state, 1);
  const team::Team team = team::Team::BLUE;
  PositionVelocityState pvs = simulator.GetWorldState(team);
  state::WorldState world_state(&pvs, team);
  logger::NetLogger logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
  PositionVelocityState::RobotPositionVelocity last_robot
      = world_state.GetOurRobotPosition(0);

  state::SoccerState soccer_state(world_state, team);

  while (true) {
    RadioProtocolWrapper command_wrapper;
    for (size_t i = 0; i < kRobots; ++i) {
      PositionVelocityState::RobotPositionVelocity robot
        = world_state.GetOurRobotPosition(0);
      if (fabs(robot.position.angle) < 3.0) {
        RadioProtocolCommand command;
        command.set_robot_id(i);
        command.set_velocity_x(0.0);
        command.set_velocity_y(0.0);
        command.set_velocity_r(1.0);
        *(command_wrapper.add_command()) = command;
      } else {
        RadioProtocolCommand command;
        command.set_robot_id(i);
        command.set_velocity_x(2);
        command.set_velocity_y(0.0);
        command.set_velocity_y(0.0);
        *(command_wrapper.add_command()) = command;
      }
    }
    simulator.SimulateStep(command_wrapper);
    pvs = simulator.GetWorldState(team);
    world_state.UpdateState(&logger);
    PositionVelocityState::RobotPositionVelocity robot
        = world_state.GetOurRobotPosition(0);
    const float movement = (.5 * 1000) * time_step;
    logger.LogPrint("Robot Position: %f, %f, %f",
                    robot.position.translation.x(),
                    robot.position.translation.y(),
                    robot.position.angle);
    logger.LogPrint("Robot Velocity: %f, %f", robot.velocity.translation.x(),
                                              robot.velocity.translation.y());
    logger.LogPrint("Robot Movement: %f", robot.position.translation.x() -
                                           last_robot.position.translation.x());
    logger.LogPrint("Expected Movement: %f", movement);
    last_robot = robot;
    logger.SetWorldState(world_state);
    logger.SendData();
    logger.Clear();
    Sleep(time_step);
  }
}
