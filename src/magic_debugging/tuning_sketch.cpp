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
  map<string, float> base_parameters;
  srtr::SolveWithBlocks(&c,
                  state_machines,
                  data,
                  &base_parameters,
                  &lowers);
}

// void TestEffects(map<string, float> epsilons,
//                  map<string, float> base_parameters,
//                  const vector<PossibleTransition>& transitions) {
//   int fixed_transition = 0;
//   int fixed_notransition = 0;
//   int unchanged_transition = 0;
//   int unchanged_notransition = 0;
//   int broken_notransition = 0;
//   int broken_transition = 0;
//   vector<string> added;
//   for (PossibleTransition transition : transitions) {
//     if (transition.human_constraint()) {
//       bool will_transition = true;
//         for (TransitionClause clause : transition.clauses()) {
//           if (std::find(added.begin(), added.end(), clause.rhs())
//               == added.end()) {
//             data_file << epsilons[clause.rhs()] << "\t";
//             added.push_back(clause.rhs());
//           }
//           if (clause.comparator().compare(">") == 0) {
//             if ( clause.lhs() <
//               (base_parameters[clause.rhs()] + epsilons[clause.rhs()])) {
//               will_transition = false;
//             }
//           }
//           if (clause.comparator().compare("<") == 0) {
//             if ( clause.lhs() >
//               (base_parameters[clause.rhs()] + epsilons[clause.rhs()])) {
//               will_transition = false;
//             }
//           }
//         }
//       if (will_transition && transition.should_transition()) {
//         fixed_transition++;
//       } else if (!will_transition && !transition.should_transition()) {
//         fixed_notransition++;
//       } else if (!will_transition && transition.should_transition()) {
//         unchanged_transition++;
//       } else {
//         unchanged_notransition++;
//       }
//     } else {
//       bool will_transition = true;
//         for (TransitionClause clause : transition.clauses()) {
//           if (std::find(added.begin(), added.end(), clause.rhs())
//               == added.end()) {
//             data_file << epsilons[clause.rhs()] << "\t";
//             added.push_back(clause.rhs());
//           }
//           if (clause.comparator().compare(">") == 0) {
//             if ( clause.lhs() <
//               (base_parameters[clause.rhs()] + epsilons[clause.rhs()])) {
//               will_transition = false;
//             }
//           }
//           if (clause.comparator().compare("<") == 0) {
//             if ( clause.lhs() >
//               (base_parameters[clause.rhs()] + epsilons[clause.rhs()])) {
//               will_transition = false;
//             }
//           }
//         }
//       if (will_transition && transition.should_transition()) {
//         unchanged_transition++;
//       } else if (!will_transition && !transition.should_transition()) {
//         unchanged_notransition++;
//       } else if (!will_transition && transition.should_transition()) {
//         broken_transition++;
//       } else {
//         broken_notransition++;
//       }
//     }
//   }
//   data_file << unchanged_transition << "\t"
//       << "\t" << fixed_transition << "\t" << broken_transition << "\t";
//   data_file << unchanged_notransition << "\t"
//       << "\t" << fixed_notransition << "\t" << broken_notransition << "\n";
// }
//
// void WriteTimeData(const vector<int>& counts,
//                    const vector<double>& time) {
//   std::ofstream myfile;
//   myfile.open("tuning_timing.txt");
//   for (size_t i = 0; i < counts.size(); ++i) {
//   myfile << counts[i] << "\t" << time[i] << std::endl;
//   }
//   myfile.close();
// }

// void TestNumCorrections() {
//   context c;
//   ReadLogger logger("medium_tuning_data.txt");
//   logger.BuildIndex();
//   // Read in tuning data from a log file.
//   vector<PossibleTransition> data;
//   vector<StateMachineData> state_machines;
//   std::cout << "Reading Log Files" << std::endl;
//   // Machine to tune is pulled out here, should make cli input
//   ReadMachineData("StateMachineAttacker", &logger, &state_machines, &data);
//   vector<PossibleTransition> corrections;
//   vector<PossibleTransition> no_corrections;
//   vector<PossibleTransition> holdout;
//   vector<PossibleTransition> for_tuning;
//   std::cout << "Data size: " <<  data.size() << std::endl;
//   int holdout_size = data.size() * .4;
//   std::cout << "Holdout Size: " << holdout_size << std::endl;
//   ChunkDataset(holdout_size, data, &holdout, &for_tuning);
//   ExtractCorrections(for_tuning, &no_corrections, &corrections);
//   vector<PossibleTransition> input = no_corrections;
//   vector<PossibleTransition> remaining = corrections;
//   vector<double> times;
//   vector<int> counts;
//   int num_corrections = 0;
//   std::cout << "Num Corrections : " << corrections.size() << std::endl;
//   for (size_t i = 1; i < corrections.size(); i += 1) {
//     for (int j = 0; j < 50; ++j) {
//       std::cout << "Number of corrections: " << i
//           << " Iteration: " << j << std::endl;
//       vector<PossibleTransition> test_input = input;
//       std::random_shuffle(remaining.begin(), remaining.end());
//       vector<PossibleTransition> output;
//       ChunkDataset(i, remaining, &test_input, &output);
//       map<string, float> lowers;
//       map<string, float> base_parameters;
//       double time = Solve(&c,
//                         state_machines,
//                         test_input,
//                         &base_parameters,
//                         &lowers);
//       times.push_back(time);
//     //     num_corrections++;
//       counts.push_back(i);
// //       data_file << i << "\t" << time << "\t";
//       num_file << i << "," << j << std::endl;
//       data_file << base_parameters["RadToDeg(thresholds_angle_)"]
//           + lowers["RadToDeg(thresholds_angle_)"] << ",";
//       data_file << base_parameters["thresholds_distance_"]
//           + lowers["thresholds_distance_"] << ",";
//       data_file << base_parameters["thresholds_align_"]
//           + lowers["thresholds_align_"] << ",";
//       data_file << base_parameters["thresholds_y_prime_vel_"]
//           + lowers["thresholds_y_prime_vel_"] << ",";
//       data_file << base_parameters["RadToDeg(thresholds_angular_vel_)"]
//           + lowers["RadToDeg(thresholds_angular_vel_)"] << ",";
//       data_file << base_parameters["thresholds_relative_vel_"]
//           + lowers["thresholds_relative_vel_"] << ",";
//       data_file << 40 << ",";
//       data_file << 800 << ",";
//       data_file << 1500 << ",";
//       data_file << 100 << ",";
//       data_file << kRobotRadius * 2.0 << ",";
//       data_file << 100 << std::endl;
// //       TestEffects(lowers, base_parameters, holdout);
//       data_file << std::flush;
//       num_file << std::flush;
// //       remaining = output;
//     num_corrections++;
//     }
//   WriteTimeData(counts, times);
//   }
// }
//
// void TestNumDataPoints() {
//   context c;
//   ReadLogger logger("medium_tuning_data.txt");
//   logger.BuildIndex();
//   // Read in tuning data from a log file.
//   vector<PossibleTransition> data;
//   vector<StateMachineData> state_machines;
//   std::cout << "Reading Log Files" << std::endl;
//   // Machine to tune is pulled out here, should make cli input
//   ReadMachineData("StateMachineAttacker", &logger, &state_machines, &data);
//   const int chunk_size = 1000;
//   int count = 0;
//   while (count < 50) {
//     vector<PossibleTransition> input;
//     vector<PossibleTransition> remaining = data;
// //     std::random_shuffle(remaining.begin(), remaining.end());
//     vector<double> times;
//     vector<int> counts;
//     for (size_t i = 42000; i < data.size(); i += chunk_size) {
//       std::cout << "________ITERATION_______" << i << std::endl;
//       vector<PossibleTransition> output;
//       ChunkDataset(chunk_size, remaining, &input, &output);
//       map<string, float> lowers;
//       map<string, float> base_parameters;
//       double time = Solve(&c,
//                           state_machines,
//                           input,
//                           &base_parameters,
//                           &lowers);
//       std::cout << time << std::endl;
//       remaining = output;
//       times.push_back(time);
//       counts.push_back(static_cast<int>(input.size()));
//       data_file << count << "," << input.size() << "," << time << std::endl;
//     }
//     count++;
//   }
// }


int main(int argc, char** argv) {
  data_file.open("all_params.txt");
  num_file.open("num.txt");
  string machine_name = "PrimaryAttacker";
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

