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
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <iomanip>      // std::setw
#include <vector>
#include <set>
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <cmath>
#include <algorithm>
#include "z3++.h" //NOLINT
#include "third_party/json.hpp"

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

void TuneFromTraceFile(const string& filename,
                       const string& machine_name,
                       const string& output_file,
                       const int& max_corrections) {
  Trace trace;
  // Read Text trace from file
  std::ifstream trace_file;
  trace_file.open(filename);
  google::protobuf::io::IstreamInputStream input_stream(&trace_file);
  google::protobuf::TextFormat::Parse(&input_stream, &trace);
  context c;

  // Read in trace data to vectors.
  vector<PossibleTransition> transitions;
  vector<StateMachineData> state_machines;
  std::cout << "Trace size: " << trace.trace_elements_size() << std::endl;
  const int num_corrections = std::min(max_corrections,
                                       trace.trace_elements_size());
  for (int i = 0; i < num_corrections; ++i) {
    StateMachineData data = trace.trace_elements(i);
    state_machines.push_back(data);
    if (machine_name.compare(data.machine_name()) == 0) {
      for (PossibleTransition transition : data.transitions()) {
          transitions.push_back(transition);
        }
    }
  }
  std::cout << "Calling SRTR" << std::endl;
  // Solve for the final adjustments.
  // The current version will output the adjustments to stdout.
  if (num_corrections > 0) {
    map<string, float> lowers;
    map<string, MapFieldEntry> base_parameters;
    nlohmann::json parameters = srtr::SolveWithBlocks(&c,
                                                      state_machines,
                                                      transitions,
                                                      &base_parameters,
                                                      &lowers);

    std::ofstream json_file(output_file);
    std::cout << std::setw(4) << parameters << std::endl;
    json_file << std::setw(4) << parameters << std::endl;
  }
}

int main(int argc, char** argv) {
  std::string machine_name = "PassFailPrimaryAttacker";
  std::string file_name = "PassFailPrimaryAttacker";
  if (argc == 2) {
    file_name = argv[1];
  }
  std::cout << "Machine Name: " << machine_name << std::endl;
//   const double start_time = GetWallTime();
  const int all_corrections = INT_MAX;
  TuneFromTraceFile(file_name,
                    machine_name,
                    "brass_srtr.json",
                    all_corrections);
  const int starved_corrections = 3;
  TuneFromTraceFile(file_name,
                    machine_name,
                    "brass_srtr_starved.json",
                    starved_corrections);
//   const double end_time = GetWallTime();
//   std::cout << "Solver Time: " << end_time - start_time << std::endl;
  data_file.close();
  return 0;
}

