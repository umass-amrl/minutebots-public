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
#include <vector>
#include <set>
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <cmath>

#include "third_party/json.hpp"
#include "tuning_data.pb.h"
#include "srtr/srtr.h"

using std::map;
using MinuteBotsProto::StateMachineData;
using MinuteBotsProto::MapFieldEntry;
using MinuteBotsProto::TransitionClause;
using MinuteBotsProto::PossibleTransition;
using MinuteBotsProto::Trace;

bool FindDivergence(const string& nominal_filename,
                    const string& degraded_filename,
                    StateMachineData* correction) {
  // Read in a corresponding nominal and degraded trace
  Trace nominal_trace, degraded_trace;
  // Read Text trace from file
  std::ifstream nominal_file, degraded_file;
  nominal_file.open(nominal_filename);
  degraded_file.open(degraded_filename);
  google::protobuf::io::IstreamInputStream nominal_stream(&nominal_file);
  google::protobuf::io::IstreamInputStream degrade_stream(&degraded_file);
  google::protobuf::TextFormat::Parse(&nominal_stream, &nominal_trace);
  google::protobuf::TextFormat::Parse(&degrade_stream, &degraded_trace);

  for (int i = 1; i < nominal_trace.trace_elements_size(); ++i) {
     StateMachineData nominal_element = nominal_trace.trace_elements(i);
     StateMachineData* degraded_element =
        degraded_trace.mutable_trace_elements(i);
     StateMachineData* last_degraded =
        degraded_trace.mutable_trace_elements(i);
     // May need to dig down into the transitions instead
     // this may set the transition one frame too late.
     if (nominal_element.state() != degraded_element->state()) {
       for (int j = 0; j < last_degraded->transitions_size(); ++j) {
         PossibleTransition* transition =
            last_degraded->mutable_transitions(j);
         if (transition->potential_state() == nominal_element.state()) {
           transition->set_human_constraint(true);
           transition->set_should_transition(true);
           std::cout << "Correction: " << "should " << nominal_element.state() << std::endl;
           *correction = *degraded_element;
           return true;
         }
       }
     }
  }
  return false;
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Expects two input file paths,"  \
        "nominal_trace, and degraded_trace" << std::endl;
    return 1;
  }
  // Find the point of divergence and make a correction there.
  Trace correction_trace;
  string nominal_filename = argv[1];
  string degraded_filename = argv[2];
  StateMachineData correction;
  const bool diverged = FindDivergence(nominal_filename,
                                       degraded_filename,
                                       &correction);
  if (diverged) {
    *(correction_trace.add_trace_elements()) = correction;
  }

  // Write correction trace to a file
  string trace_file_name = "scripts/srtr/brass/results/correction_trace.txt";
  std::ofstream trace_file;
  trace_file.open(trace_file_name, std::ofstream::out | std::ofstream::app);
  google::protobuf::io::OstreamOutputStream output_stream(&trace_file);
  google::protobuf::TextFormat::Print(correction_trace, &output_stream);
}

