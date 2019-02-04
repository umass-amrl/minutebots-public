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
#include "tactics/role_assignment.h"

using datastructures::OptionalValue;

namespace role {

const unsigned int kMaxRobots = 10;

RoleAssignment::RoleAssignment() : role_to_robot_map(kMaxRobots) {}

RoleAssignment::~RoleAssignment() {}

void RoleAssignment::SetRollAssignment(RollId roll, RobotId robot) {
  role_to_robot_map.Insert(roll, robot);
}

const RobotId RoleAssignment::GetRollAssignment(RollId roll) const {
  const OptionalValue<RobotId>& opt = role_to_robot_map.Read(roll);
  if (!opt.has_value) {
    LOG(FATAL) << "Tried to find robotId for RollId " << roll << "\n";
  }
  return opt.val;
}
}  // namespace role
