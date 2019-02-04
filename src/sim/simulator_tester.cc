// Copyright 2017 - 2018 jaholtz@cs.umass.edu
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

#include "sim/simulator.h"
#include "constants/constants.h"
#include "logging/logger.h"
#include "state/soccer_state.h"
#include "state/world_state.h"

using simulator::WorldState;
using simulator::Simulator;
using state::PositionVelocityState;

const double noise_level = 0.00;
std::default_random_engine generator;
std::normal_distribution<double> distribution(1.0, noise_level);


int main() {
  const double time_step = 1.0f / 60.0f;
  const int kRobots = 1;
  WorldState state(kRobots, time_step);
  Simulator simulator(time_step, &state);
  const team::Team team = team::Team::BLUE;
  PositionVelocityState pvs = simulator.GetWorldState(team);
  state::WorldState world_state(&pvs, team);
  logger::NetLogger logger(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
  state::SoccerState soccer_state_(world_state, team);

  while (true) {
    RadioProtocolWrapper command_wrapper;
    for (size_t i = 0; i < kRobots; ++i) {
      RadioProtocolCommand command;
      command.set_robot_id(i);
      command.set_velocity_x(1.0);
      command.set_velocity_y(0.0);
      command.set_velocity_y(0.1);
      *(command_wrapper.add_command()) = command;
    }
    simulator.SimulateStep(command_wrapper);
    pvs = simulator.GetWorldState(team);
    world_state.UpdateState(&logger);
    logger.SetWorldState(world_state);
    logger.SendData();
  }
}
