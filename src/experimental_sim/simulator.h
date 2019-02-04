// Copyright 2018 jaholtz@cs.umass.edu
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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tuning_data.pb.h"
#include "state/world_state.h"
#include "state/position_velocity_state.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "radio_protocol_wrapper.pb.h"
#include "soccer_logging.pb.h"
using std::string;

#ifndef SRC_EXPERIMENTAL_SIM_SIMULATOR_H_
#define SRC_EXPERIMENTAL_SIM_SIMULATOR_H_

namespace experimental_simulator {

class Simulator {
 public:
  Simulator();

  virtual std::vector<SSLVisionProto::SSL_WrapperPacket>
      GetSSLWrapperPackets();

  virtual state::PositionVelocityState GetWorldState(const team::Team team);

  virtual void
      SimulateStep(const RadioProtocolWrapper& RadioProtocolCommand);

  virtual void SetFactors(MinuteBotsProto::FactorSettings settings);

  virtual MinuteBotsProto::FactorTuningData GetFactorChanges() = 0;

  virtual Simulator* Copy() = 0;
};

}  // namespace experimental_simulator

#endif  // SRC_EXPERIMENTAL_SIM_SIMULATOR_H_
