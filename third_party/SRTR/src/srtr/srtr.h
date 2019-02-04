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
#include <stdlib.h>     /* atof */
#include <vector>
#include <set>
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <cmath>
#include "z3++.h" //NOLINT
#include "third_party/json.hpp"
#include "tuning_data.pb.h"


using z3::optimize;
using z3::expr;
using z3::context;
using z3::solver;
using z3::sat;
using std::vector;
using std::map;
using MinuteBotsProto::StateMachineData;
using MinuteBotsProto::MapFieldEntry;
using MinuteBotsProto::TransitionClause;
using MinuteBotsProto::PossibleTransition;
using std::string;
using std::endl;
// std::ofstream data_file;
// std::ofstream num_file;

#ifndef SRC_SRTR_SRTR_H_
#define SRC_SRTR_SRTR_H_

namespace srtr {

void GetParameters(context* c,
                   optimize* opt,
                   const vector<StateMachineData>& machines,
                   map<string, float>* base_parameters,
                   vector<string>* param_names,
                   map<string, expr>* epsilons,
                   map<string, expr>* absolutes,
                   vector<optimize::handle>* handles);

nlohmann::json SolveWithBlocks(context* c,
             const vector<StateMachineData>& machines,
             const vector<PossibleTransition>& data,
             map<string, float>* params,
             map<string, float>* lowers);

void TuneFromTraceFile(const string& filename, const string&  machine_name);

}  // namespace srtr

#endif  // SRC_SRTR_SRTR_H_
