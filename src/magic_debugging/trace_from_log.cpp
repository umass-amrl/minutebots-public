// Copyright 2017-2019 jaholtz@cs.umass.edu
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
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
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
using MinuteBotsProto::Trace;
using logger::ReadLogger;
std::ofstream data_file;
std::ofstream num_file;

void ReadMachineData(const string& machine_name,
                     ReadLogger* logger,
                     vector<StateMachineData>* state_machines,
                     vector<PossibleTransition>* data_vector) {
  int transition_count = 0;
  for (int i = 0; i < logger->GetMapSize(); ++i) {
    SoccerDebugMessage message = logger->GetNextMessage().soccer_debug();
    for (int j = 0; j < message.tuning_data_size(); ++j) {
      StateMachineData data = message.tuning_data(j);
      state_machines->push_back(data);
      if (machine_name.compare(data.machine_name()) == 0) {
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

Trace TraceFromLog(const string& machine_name, const string& filename) {
  context c;
  ReadLogger logger(filename);
  logger.BuildIndex();

  Trace trace;
  // Iterate over the log and pull out StateMachineData.
  for (int i = 0; i < logger.GetMapSize(); ++i) {
    SoccerDebugMessage message = logger.GetNextMessage().soccer_debug();
    for (int j = 0; j < message.tuning_data_size(); ++j) {
      StateMachineData data = message.tuning_data(j);
      std::cout << data.machine_name() << std::endl;
      if (machine_name.compare(data.machine_name()) == 0) {
        *(trace.add_trace_elements()) = data;
      }
    }
  }
  return trace;
}

int main(int argc, char** argv) {
  string machine_name = "PassFailPrimaryAttacker";
  string filename = "kick_tuning_data.txt";
  if (argc >= 2) {
    machine_name = argv[1];
  }
  if (argc >= 3) {
    filename = argv[2];
  }
  printf("Gathering %s trace from %s \n",
         machine_name.c_str(),
         filename.c_str());
  Trace trace = TraceFromLog(machine_name, filename);
  std::ofstream trace_file;
  trace_file.open("extracted_trace.txt");
  google::protobuf::io::OstreamOutputStream output_stream(&trace_file);
  google::protobuf::TextFormat::Print(trace, &output_stream);
  return 0;
}
