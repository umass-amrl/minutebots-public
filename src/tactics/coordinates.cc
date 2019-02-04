// Copyright 2017 rezecib@gmail.com
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
#include "tactics/coordinates.h"

#include <string>
#include <vector>

#include "constants/constants.h"
#include "state/world_state.h"

STANDARD_USINGS;
using state::WorldState;

namespace coordinates {

static vector<string> Split(const string& s);

Eigen::Vector2f ParseCoordinates(const WorldState& soccer_state,
                                 const string& coordinates) {
  // TODO(rezecib): support more types of coordinates
  vector<string> parts = Split(coordinates);
  if (parts.at(0).compare("B") == 0) {
    // TODO(rezecib): ball-relative coordinates
    return Vector2f(stof(parts.at(0)), stof(parts.at(1)));
  } else {
    return Vector2f(stof(parts.at(0)), stof(parts.at(1)));
  }
}

static vector<string> Split(const string& s) {
  vector<string> elems;
  std::stringstream ss;
  ss.str(s);
  string item;
  while (std::getline(ss, item, '_')) {
    elems.push_back(item);
  }
  return elems;
}

}  // namespace coordinates
