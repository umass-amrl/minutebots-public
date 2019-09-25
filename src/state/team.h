// Copyright 2017-2019 kvedder@umass.edu
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

#include "soccer_logging.pb.h"

#ifndef SRC_STATE_TEAM_H_
#define SRC_STATE_TEAM_H_

namespace team {
enum class Team {
  BLUE,
  YELLOW,
};

bool operator==(const Team& t1, const MinuteBotsProto::RobotState_Team& t2);

bool operator==(const MinuteBotsProto::RobotState_Team& t1, const Team& t2);

}  // namespace team

#endif  // SRC_STATE_TEAM_H_
