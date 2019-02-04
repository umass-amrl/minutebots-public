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

#include <string>

#include "state/world_state.h"

#ifndef SRC_TACTICS_COORDINATES_H_
#define SRC_TACTICS_COORDINATES_H_

namespace coordinates {
  Eigen::Vector2f ParseCoordinates(const state::WorldState& soccer_state,
                                   const std::string& coordinates);
}  // namespace coordinates

#endif  // SRC_TACTICS_COORDINATES_H_
