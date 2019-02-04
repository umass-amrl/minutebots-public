// Copyright 2017 slane@cs.umass.edu, kvedder@umass.edu
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

#include "constants/constants.h"
#include "datastructures/better_map.h"

#ifndef SRC_TACTICS_ROLE_ASSIGNMENT_H_
#define SRC_TACTICS_ROLE_ASSIGNMENT_H_

namespace role {

class RoleAssignment {
 public:
  RoleAssignment();
  ~RoleAssignment();

  void SetRollAssignment(RollId roll, RobotId robot);

  const RobotId GetRollAssignment(RollId roll) const;

 private:
  datastructures::BetterMap<RobotId> role_to_robot_map;
};

}  // namespace role

#endif  // SRC_TACTICS_ROLE_ASSIGNMENT_H_
