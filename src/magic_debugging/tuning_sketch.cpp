// Copyright 2011-2018 jaholtz@cs.umass.edu
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

#include <stdlib.h>     /* atof */
#include <vector>
#include <set>
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <cmath>
#include "z3++.h" //NOLINT
#include "tuning_data.pb.h"
#include "logging/logger.h"
#include "util/timer.h"
#include "srtr/srtr.h"

using z3::optimize;
using z3::expr;
using z3::context;
using z3::solver;
using z3::sat;
using std::vector;
using std::map;
using MinuteBotsProto::StateMachineData;
using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::MapFieldEntry;
using MinuteBotsProto::TransitionClause;
using MinuteBotsProto::PossibleTransition;
using logger::ReadLogger;
std::ofstream data_file;
std::ofstream num_file;

void ReadMachineData(const string& machine_name,
                                ReadLogger* logger,
                                vector<StateMachineData>* state_machines,
                                vector<PossibleTransition>* data_vector) {
  int transition_count = 0;
  std::cout << logger->GetMapSize() << std::endl;
  for (int i = 0; i < logger->GetMapSize(); ++i) {
    SoccerDebugMessage message = logger->GetNextMessage().soccer_debug();
    for (int j = 0; j < message.tuning_data_size(); ++j) {
      StateMachineData data = message.tuning_data(j);
      state_machines->push_back(data);
      std::cout << machine_name << std::endl;
      if (machine_name.compare(data.machine_name()) == 0) {
        std::cout << "Machine: " << data.machine_name() << std::endl;
        for (PossibleTransition transition : data.transitions()) {
          if (transition.should_transition()) {
            transition_count++;
          }
          data_vector->push_back(transition);
        }
      }
    }
  }
}

void ExtractCorrections(const vector<PossibleTransition>& input,
                        vector<PossibleTransition>* no_corrections,
                        vector<PossibleTransition>* corrections) {
  for (size_t i = 0; i < input.size(); ++i) {
    if (input[i].human_constraint()) {
      corrections->push_back(input[i]);
    } else {
      no_corrections->push_back(input[i]);
    }
  }
  std::cout << "Num Corrections: " << corrections->size() << std::endl;
}

void ChunkDataset(const int& chunk_size,
                  const vector<PossibleTransition>& input,
                  vector<PossibleTransition>* chunk,
                  vector<PossibleTransition>* remaining) {
  int count = chunk_size;
  for (size_t i = 0; i < input.size(); ++i) {
    if (count > 0) {
      chunk->push_back(input[i]);
    } else {
      remaining->push_back(input[i]);
    }
    count--;
  }
}

void TuneMachine(const string& machine_name) {
  context c;
  ReadLogger logger("kick_tuning_data.txt");
  logger.BuildIndex();
  // Read in tuning data from a log file.
  vector<PossibleTransition> data;
  vector<StateMachineData> state_machines;
  std::cout << "Reading Log Files" << std::endl;
  // Machine to tune is pulled out here, should make cli input
  ReadMachineData(machine_name, &logger, &state_machines, &data);
  map<string, float> lowers;
  map<string, MapFieldEntry> base_parameters;
  srtr::SolveWithBlocks(&c,
                  state_machines,
                  data,
                  &base_parameters,
                  &lowers);
}

int main(int argc, char** argv) {
  data_file.open("all_params.txt");
  num_file.open("num.txt");
  string machine_name = "PassFailPrimaryAttacker";
  if (argc == 2) {
    machine_name = argv[1];
  }
    std::cout << "Machine Name: " << machine_name << std::endl;
    const double start_time = GetWallTime();
    TuneMachine(machine_name);
    const double end_time = GetWallTime();
    std::cout << "Solver Time: " << end_time - start_time << std::endl;
    data_file.close();
//   srtr::TuneFromTraceFile(machine_name);
  return 0;
}

