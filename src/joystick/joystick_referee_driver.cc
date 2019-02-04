// Copyright 2017-2018 jaholtz@cs.umass.edu
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

// Joystick Driver main file
// Takes a robot ID as an argument
// Control a robot via an XBox Controller

#include <stdlib.h>
#include <iostream>
#include <string>

#include "constants/constants.h"
#include "joystick/joystick.h"
#include "net/netraw.h"
#include "radio_protocol_wrapper.pb.h"
#include "referee.pb.h"
#include "util/timer.h"

STANDARD_USINGS;
using joystick::Joystick;
using net::UDPMulticastServer;
using std::cout;
using std::cerr;
using std::endl;
using std::atoi;
using MinuteBotsProto::SSL_Referee;

int main(int argc, char** argv) {
  if (argc != 3) {
    cout << "Usage: joystick_referee_driver";
    cout << "<Joystick ID> <Refbox Port>\n";
    return 1;
  }
  int joystick_id = atoi(argv[1]);

  Joystick* joystick = new Joystick();
  if (!joystick->Open(joystick_id)) {
    cerr << "ERROR: Unable to open joystick!" << endl;
    return 1;
  }

  RateLoop time_helper(60);

  const string udp_address = DATA_STREAM_REF_IP;
  const int udp_port = atoi(argv[2]);
  UDPMulticastServer udp_server;
  cout << "Opening HandleExecution multicast on IP: " << udp_address
       << " with Port: " << udp_port << endl;
  if (!udp_server.Open(udp_address, udp_port, false)) {
    cerr << "Error opening UDP port of Executor's "
         << "HandleExecution thread, exiting." << endl;
  } else {
    cout << "Opening UDP of Executor's HandleExecution thread "
         << "successful!" << endl;
  }
  SSL_Referee message;
  SSL_Referee::TeamInfo* blue = message.mutable_blue();
  SSL_Referee::TeamInfo* yellow = message.mutable_yellow();
  blue->set_name("blue");
  blue->set_score(0);
  blue->set_red_cards(0);
  blue->set_yellow_cards(0);
  blue->set_timeouts(0);
  blue->set_timeout_time(0);
  blue->set_goalie(0);
  yellow->set_name("yellow");
  yellow->set_score(0);
  yellow->set_red_cards(0);
  yellow->set_yellow_cards(0);
  yellow->set_timeouts(0);
  yellow->set_timeout_time(0);
  yellow->set_goalie(0);
  message.set_packet_timestamp(0);
  message.set_command_timestamp(0);
  message.set_stage(MinuteBotsProto::SSL_Referee_Stage_NORMAL_FIRST_HALF);
  SSL_Referee::Point* placement_point = message.mutable_designated_position();
  placement_point->set_x(1000);
  placement_point->set_y(1000);
  int i = 0;
  while (true) {
    i++;
    SSL_Referee::Command temp_command = message.command();
    bool joystick_input = joystick->MakeRefCommand(&temp_command);
    message.set_command(temp_command);
    if (joystick_input) {
      message.set_command_counter(i);
      if (temp_command == MinuteBotsProto::SSL_Referee_Command_HALT) {
        printf("Halt\n");
      }
      if (!udp_server.SendProtobuf(message)) {
        cerr << "Send output error" << endl;
      }
    }
    time_helper.Sleep();
  }

  joystick->Close();
}
