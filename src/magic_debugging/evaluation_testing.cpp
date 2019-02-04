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

float Test(const float& x, const float& y) {
  const float threshold = 6.0;
  if (x < threshold) {
    return x + y;
  } else {
    return x - y;
  }
}

int main(int argc, char** argv) {
  const float x = 5;
  const float y = 10;
  float z = Test(x, y);
  return 0;
}

