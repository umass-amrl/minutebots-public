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

#include <stdlib.h>     /* atof */
#include <vector>
#include <set>
#include <utility>
#include <map>
#include <fstream>
#include <string>
#include <cmath>
#include "dreal/dreal.h" //NOLINT
#include <experimental/optional> //NOLINT
#include "tuning_data.pb.h"
#include "logging/logger.h"
#include "util/timer.h"


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
using std::experimental::optional;
std::ofstream data_file;
std::ofstream num_file;

void ReadMachineData(const string& machine_name,
                                ReadLogger* logger,
                                vector<StateMachineData>* state_machines,
                                vector<PossibleTransition>* data_vector) {
  int transition_count = 0;
  for (int i = 0; i < logger->GetMapSize(); ++i) {
//     std::cout << "Logger Size: " << logger->GetMapSize() << std::endl;
    SoccerDebugMessage message = logger->GetNextMessage().soccer_debug();
    for (int j = 0; j < message.tuning_data_size(); ++j) {
      StateMachineData data = message.tuning_data(j);
//       PrintStateMachineData(data);
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
          absolute = ite(temp >= 0,
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

double SolveWithBlocksDreal(context* c,
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
  dreal::Formula problem;
  dreal::Expression sum_of_epislons;
  dreal::Variable test;
  for (size_t i = 0; i < data.size(); ++i) {
    PossibleTransition data_point = data[i];
    dreal::Formula full_formula;
    // For each block in that transition (builds full boolean clause)
    for (auto k = 0; k < data_point.blocks_size(); ++k) {
      const bool constrained = data_point.human_constraint();
      MinuteBotsProto::TransitionBlock block = data_point.blocks(k);
      dreal::Formula formula_block;
      // For each clause in that block
      for (auto j = 0; j < block.clauses_size(); ++j) {
        TransitionClause clause = block.clauses(j);
        const float lhs = clause.lhs();
        const string rhs = clause.rhs();
        if (constrained) {
          tuned_parameters.insert(rhs);
        }
        dreal::Formula current_clause;
        // Identify which comparator is used and add the statement
        if (clause.comparator().compare(">") == 0) {
          dreal::Variable temp{rhs};
          current_clause = lhs > base_parameters[rhs] + temp;
          if (constrained) {
            sum_of_epislons = sum_of_epislons + abs(temp);
          }
        } else if (clause.comparator().compare("<") == 0) {
          dreal::Variable temp{rhs};
          current_clause = lhs < base_parameters[rhs] + temp;
          if (constrained) {
            sum_of_epislons = sum_of_epislons + abs(temp);
            test = temp;
          }
        }  // etc for all handled comparators right now only handles ">" and "<"

        // Combine this statement with the rest of the block
        if (j == 0) {
          formula_block = current_clause;
        } else {
          if (clause.and_()) {
            formula_block = formula_block && current_clause;
          } else {
            formula_block = formula_block || current_clause;
          }
        }
      }
      // Combine this block with the other blocks
      if (k == 0) {
        full_formula = formula_block;
      } else if (block.and_()) {
        full_formula = full_formula && formula_block;
      } else {
        full_formula = full_formula || formula_block;
      }
    }

    // Add the data point if it's a human constraint
    if (data_point.human_constraint()) {
      if (data_point.should_transition()) {
        problem = problem && full_formula;
      } else {
        problem = problem && !full_formula;
      }
    }
  }

  vector<expr> epsilon_values;
  for (auto param : tuned_parameters) {
    optimize::handle handle = opt.minimize(absolutes.at(param));
    handles.push_back(handle);
    epsilon_values.push_back(tuning.at(param));
  }

  double start_time = GetMonotonicTime();
  std::cout << test << std::endl;
  std::cout << std::endl;
  optional<dreal::Box> result = dreal::Minimize(test,
                                                problem,
                                                0.001);
  if (result) {
    std::cout << problem << " is delta-sat:\n" << *result << std::endl;
  } else {
    std::cout << problem << " is unsat." << std::endl;
  }
  double end_time = GetMonotonicTime();
  double execution_time = end_time - start_time;
  return execution_time;
}

void TestDreal() {
  dreal::Expression to_minimize;
  dreal::Variable align{"align"};
  dreal::Variable angle {"angle"};
  dreal::Formula problem;
  // First clause
  //   to_minimize = to_minimize + abs(align);
  //   problem = 815.7918 < 7 + align;
  //
  //   // Second Clause
  //   dreal::Variable angle{"angle"};
  //   problem = problem && (2.4388 < 57 + angle);
  // //   to_minimize = abs(angle);
  //

  dreal::Variable test{"test"};
  problem = align < 10 && align > 5 && angle > 2
      && angle < 2000 && test == abs(align) + abs(angle);

  std::cout << to_minimize << std::endl;
  optional<dreal::Box> result;
  result = dreal::Minimize(test,
                          problem,
                          .0001);
  //   std::cout << "MINIMIZATION" << std::endl;
  //   std::cout << "---------------------------------" << std::endl;
  //   if (result) {
  //     std::cout << problem << " is delta-sat:\n" << *result << std::endl;
  //   } else {
  //     std::cout << problem << " is unsat." << std::endl;
  //   }
  //   std::cout << "---------------------------------" << std::endl;
  result = dreal::CheckSatisfiability(problem,
                                      .0001);
  std::cout << "Satisfiability" << std::endl;
  std::cout << "---------------------------------" << std::endl;
  if (result) {
    std::cout << problem << " is delta-sat:\n" << *result << std::endl;
  } else {
    std::cout << problem << " is unsat." << std::endl;
  }
}

void TuneMachineDreal(const string& machine_name) {
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
  map<string, float> base_parameters;
  double time = SolveWithBlocksDreal(&c,
                                state_machines,
                                data,
                                &base_parameters,
                                &lowers);
  std::cout << time << std::endl;
}

int main(int argc, char** argv) {
  //   string machine_name = "StateMachineAttacker";
  //   TuneMachineDreal(machine_name);
  TestDreal();
  return 0;
}

