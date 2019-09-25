// Copyright 2019 jaholtz@cs.umass.edu
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

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "util/random.h"
#include "simplecar_sim/simplecar_state.h"

#ifndef SRC_SIMPLECAR_SIM_SIMPLECAR_SIM_H_
#define SRC_SIMPLECAR_SIM_SIMPLECAR_SIM_H_

namespace simplecar {
class SimplecarSim {
 public:
    SimplecarSim(const float& step_time,
                 const string& config_file);

    SimplecarSim();

    ~SimplecarSim();

    // Returns the SimpleCarState
    SimplecarState GetWorldState();

    // Simulates a single time step.
    void SimulateStep();
    void SimulateStep(const SimplecarCommand& command);
 private:
    // The amount of time to simulate in an update.
    float step_time_;
    double current_time_;
    int frame_index_;
    // Contains the simulated world_state_
    SimplecarState world_state_;
};
}  // namespace simplecar

#endif  // SRC_SIMPLECAR_SIM_SIMPLECAR_SIM_H_
