// Copyright 2017-2018 jaholtz@cs.umass.edu
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

#include <iomanip>      // std::setw
#include "srtr/srtr.h"
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "third_party/json.hpp"
using MinuteBotsProto::Trace;
using MinuteBotsProto::StateMachineData;
using MinuteBotsProto::Trace;
using std::fstream;
using std::ios;
using std::cerr;
using std::cout;
using std::endl;

namespace srtr {

void GetParameters(context* c,
                   optimize* opt,
                   const vector<StateMachineData>& machines,
                   map<string, float>* base_parameters,
                   vector<string>* param_names,
                   map<string, expr>* epsilons,
                   map<string, expr>* absolutes,
                   vector<optimize::handle>* handles) {
  for (auto data : machines) {
    expr sum = c->real_val("sum");
    bool uninit = true;
    for (int i = 0; i < data.tuneable_params_size(); ++i) {
      MapFieldEntry entry = data.tuneable_params(i);
      if (!base_parameters->count(entry.key())) {
        (*base_parameters)[entry.key()] = entry.value();
        expr temp = c->real_const(entry.key().c_str());
        param_names->push_back(entry.key());
        expr absolute = c->real_val("absolute");
        absolute = ite(temp >= c->real_val(0),
                      temp,
                      -temp);
        if (uninit) {
          sum = absolute;
          uninit = false;
        } else {
          sum = sum + absolute;
        }
        absolutes->insert(std::pair<string, expr>(entry.key(), absolute));
        epsilons->insert(std::pair<string, expr>(entry.key(), temp));
      }
    }
  }
}

nlohmann::json SolveWithBlocks(context* c,
             const vector<StateMachineData>& machines,
             const vector<PossibleTransition>& data,
             map<string, float>* params,
             map<string, float>* lowers) {
  // The thresholds we will tune and their epsilons
  map<string, float> base_parameters;
  map<string, expr> tuning;
  vector<string> param_names;
  vector<optimize::handle> handles;
  map<string, expr> absolutes;
  // Declare solvers
  solver solver(*c);
  optimize opt(*c);

  GetParameters(c,
                &opt,
                machines,
                &base_parameters,
                &param_names,
                &tuning,
                &absolutes,
                &handles);

  *params = base_parameters;
  // For each possible transition add a statement
  std::set<string> tuned_parameters;
  for (size_t i = 0; i < data.size(); ++i) {
    expr transition = c->bool_val("transition");
    expr neg_transition = c->bool_val("negative_transition");
    PossibleTransition data_point = data[i];
    // For each block in that transition (builds full boolean clause)
    for (auto k = 0; k < data_point.blocks_size(); ++k) {
      const bool constrained = data_point.human_constraint();
      MinuteBotsProto::TransitionBlock block = data_point.blocks(k);
      expr block_transition = c->bool_val("block_transition");
      // For each clause in that block
      for (auto j = 0; j < block.clauses_size(); ++j) {
        TransitionClause clause = block.clauses(j);
        const float lhs = clause.lhs();
        const string rhs = clause.rhs();
        if (constrained) {
          tuned_parameters.insert(rhs);
        }
        expr clause_bool = c->bool_val("case");

        // Identify which comparator is used and add the statement
        if (clause.comparator().compare(">") == 0) {
          expr temp = c->real_val(std::to_string(lhs).c_str())
              > c->real_val(std::to_string(base_parameters[rhs]).c_str())
                  + tuning.at(rhs);
          clause_bool = temp;
        } else if (clause.comparator().compare("<") == 0) {
          expr temp = c->real_val(std::to_string(lhs).c_str())
              < c->real_val(std::to_string(base_parameters[rhs]).c_str())
                  + tuning.at(rhs);
          clause_bool = temp;
        }  // etc for all handled comparators right now only handles ">" and "<"

        // Combine this statement with the rest of the block
        if (j == 0) {
          block_transition = clause_bool;
        } else {
          if (clause.and_()) {
            block_transition = block_transition && clause_bool;
          } else {
            block_transition = block_transition || clause_bool;
          }
        }
      }
      // Combine this block with the other blocks
      if (k == 0) {
        transition = block_transition;
      } else if (block.and_()) {
        transition = transition && block_transition;
      } else {
        transition = transition || block_transition;
      }
    }

    // Add the data point if it's a human constraint
    if (data_point.human_constraint()) {
      if (data_point.should_transition()) {
        opt.add(transition, 1);
      } else {
        opt.add(!transition, 1);
      }
    }
  }

  vector<expr> epsilon_values;
  for (auto param : tuned_parameters) {
    optimize::handle handle = opt.minimize(absolutes.at(param));
    handles.push_back(handle);
    epsilon_values.push_back(tuning.at(param));
  }
  z3::params p(*c);
  opt.set(p);
  z3::set_param("pp.decimal", true); // set decimal notation
  std::cout << "Starting solver" << std::endl << std::endl;
  std::cout << "---------------------------------" << std::endl;
  std::cout << "SMT2 Representation" << std::endl;
  std::cout << "---------------------------------" << std::endl;
  std::cout << opt;
  std::cout << "---------------------------------" << std::endl << std::endl;
  std::cout << "---------------------------------" << std::endl;

  std::cout << "Parameter Adjustments" << std::endl;
  std::cout << "---------------------------------" << std::endl;
  if (sat == opt.check()) {
      std::cout << opt.get_model() << std::endl;
      std::cout << opt.objectives() << std::endl;
    int i = 0;
    for (auto param : tuned_parameters) {
      // TODO(jaholtz) param_names may need to be corrected.
      std::cout << param;
      std::cout << ": " <<
          opt.get_model().get_const_interp(tuning.at(param).decl());
      std::cout << " : " <<
          opt.get_model().get_const_interp(
            tuning.at(param).decl()).get_decimal_string(4)
          << std::endl;
      i++;
    }
  }
  // Generating a JSON config file for the behavior as output.
  nlohmann::json config_json;
  for (auto const& param: base_parameters) {
    float value = param.second;
    auto map_it = tuned_parameters.find(param.first);
    if (map_it != tuned_parameters.end()) {
      // Get fractional components of delta
      const float denom = opt.get_model().get_const_interp(
          tuning.at(param.first).decl()).denominator().get_numeral_int();
      const float num = opt.get_model().get_const_interp(
          tuning.at(param.first).decl()).numerator().get_numeral_int();
      if (fabs(denom) > 0.0) {
        // Doing division ourselves, z3 can't be trusted
        const float delta = num / denom;
        value += delta;
      }
    }
    config_json[param.first] = value;
  }
  std::ofstream json_file("srtr_output.json");
  json_file << std::setw(4) << config_json << std::endl;
  return config_json;
}

void TuneFromTraceFile(const string& filename,
                       const string& machine_name) {
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
  for (int i = 0; i < trace.trace_elements_size(); ++i) {
    StateMachineData data = trace.trace_elements(i);
    state_machines.push_back(data);
    if (machine_name.compare(data.machine_name()) == 0) {
      for (PossibleTransition transition : data.transitions()) {
          transitions.push_back(transition);
        }
    }
  }
  // Solve for the final adjustments.
  map<string, float> lowers;
  map<string, float> base_parameters;
  SolveWithBlocks(&c,
                  state_machines,
                  transitions,
                  &base_parameters,
                  &lowers);
}


}  // namespace srtr

