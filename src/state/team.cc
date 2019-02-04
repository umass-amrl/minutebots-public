// Copyright 2017-2018 kvedder@umass.edu
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

#include "state/team.h"
#include <glog/logging.h>
#include "soccer_logging.pb.h"

namespace team {

bool operator==(const Team& t1, const MinuteBotsProto::RobotState_Team& t2) {
  switch (t1) {
    case Team::BLUE: {
      return (t2 == MinuteBotsProto::RobotState_Team_TEAM_BLUE);
    } break;
    case Team::YELLOW: {
      return (t2 == MinuteBotsProto::RobotState_Team_TEAM_YELLOW);
    } break;
    default: {
      return false;
    } break;
  }
}

bool operator==(const MinuteBotsProto::RobotState_Team& t1, const Team& t2) {
  switch (t1) {
    case MinuteBotsProto::RobotState_Team_TEAM_BLUE: {
      return (t2 == Team::BLUE);
    } break;
    case MinuteBotsProto::RobotState_Team_TEAM_YELLOW: {
      return (t2 == Team::YELLOW);
    } break;
    default: {
      return false;
    } break;
  }
}

}  // namespace team
