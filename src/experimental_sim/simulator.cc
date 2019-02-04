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
#include "experimental_sim/simulator.h"

using state::PositionVelocityState;
using SSLVisionProto::SSL_WrapperPacket;
using std::vector;

namespace experimental_simulator {

Simulator::Simulator() {}

vector<SSL_WrapperPacket> Simulator::GetSSLWrapperPackets() {
  const vector<SSL_WrapperPacket> empty;
  return empty;
}

PositionVelocityState Simulator::GetWorldState(const team::Team team) {
  const PositionVelocityState default_state;
  return default_state;
}

void Simulator::SetFactors(MinuteBotsProto::FactorSettings settings) {
  return;
}

MinuteBotsProto::FactorTuningData Simulator::GetFactorChanges() {
  MinuteBotsProto::FactorTuningData empty;
  return empty;
}

void Simulator::SimulateStep(const RadioProtocolWrapper& RadioProtocolCommand) {
  std::cout << "I'm pretending to simulate a step or something." << std::endl;
  return;
}


}  // namespace experimental_simulator
