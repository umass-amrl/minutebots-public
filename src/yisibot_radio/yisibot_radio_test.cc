// Copyright 2017 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
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
//========================================================================

#include <stdio.h>
#include <vector>
#include <iostream>
#include "yisibot_radio/yisibot_radio.h"
#include "util/timer.h"

int main(int argc, char** argv) {
  YisibotRobotCommand robot_0_command;
  YisibotRobotCommand robot_1_command;
  // YisibotRobotCommand robot_2_command;
  // YisibotRobotCommand robot_3_command;

  robot_0_command.id = 0;
  robot_1_command.id = 1;
  // robot_2_command.id = 2;
  // robot_3_command.id = 3;

  robot_0_command.x_velocity = 50;
  robot_1_command.x_velocity = 0;
  // robot_2_command.x_velocity = 100;
  // robot_3_command.x_velocity = 100;

  robot_0_command.y_velocity = 0;
  robot_1_command.y_velocity = 0;
  // robot_2_command.y_velocity = 0;
  // robot_3_command.y_velocity = 0;

  robot_0_command.rotational_velocity = 0;
  robot_1_command.rotational_velocity = 0;
  // robot_2_command.rotational_velocity = 0;
  // robot_3_command.rotational_velocity = 0;

  robot_0_command.shoot_power = 0;
  robot_0_command.shoot_mode = 0;
  robot_0_command.shoot = 0;

  robot_1_command.shoot_power = 0;
  robot_1_command.shoot_mode = 0;
  robot_1_command.shoot = 0;

  // robot_2_command.shoot_power = 0;
  // robot_2_command.shoot_mode = 0;
  // robot_2_command.shoot = 0;

  // robot_3_command.shoot_power = 0;
  // robot_3_command.shoot_mode = 0;
  // robot_3_command.shoot = 0;

  robot_0_command.dribble_level = 0;
  robot_0_command.dribble_direction = 0;
  robot_0_command.dribble = 0;

  robot_1_command.dribble_level = 1;
  robot_1_command.dribble_direction = 0;
  robot_1_command.dribble = 1;

  // robot_2_command.dribble_level = 0;
  // robot_2_command.dribble_direction = 0;
  // robot_2_command.dribble = 0;

  // robot_3_command.dribble_level = 0;
  // robot_3_command.dribble_direction = 0;
  // robot_3_command.dribble = 0;

  std::vector<YisibotRobotCommand> message;
  message.push_back(robot_0_command);
  message.push_back(robot_1_command);
  // message.push_back(robot_2_command);
  // message.push_back(robot_3_command);

  std::vector<YisibotRobotCommand> sent_commands;

  YisibotRadio radio;
  if (!radio.Init("/dev/ttyUSB0", 0)) {
    std::cout << "ERROR: Unable to open serial port device(s)!\n";
    return(2);
  }

  RateLoop time_helper(60);

  while (true) {
    radio.Send(message, &sent_commands);
    time_helper.Sleep();
  }

  radio.Close();

  return 0;
}

