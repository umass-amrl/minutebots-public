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

// Joystick Driver main file
// Takes a robot ID as an argument
// Control a robot via an XBox Controller

#include <stdlib.h>
#include <iostream>
#include <string>

#include "constants/constants.h"
#include "net/netraw.h"
#include "joystick/joystick.h"
#include "util/timer.h"
#include "radio_protocol_wrapper.pb.h"

STANDARD_USINGS;
using joystick::Joystick;
using net::UDPMulticastServer;
using std::cout;
using std::cerr;
using std::endl;
using std::atoi;

int main(int argc, char** argv) {
  if (argc != 3) {
    cout << "Usage: joystick_driver";
    cout << " <Robot ID> <Joystick ID>\n";
    return(1);
  }
  int robot_id = atoi(argv[1]);
  int joystick_id = atoi(argv[2]);

  Joystick* joystick = new Joystick();
  if (!joystick->Open(joystick_id)) {
    cerr << "ERROR: Unable to open joystick)!" << endl;
    return(1);
  }

  RateLoop time_helper(60);

  const string udp_address = DATA_STREAM_CMD_IP;
  const int udp_port = DATA_STREAM_CMD_PORT;
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

  while (true) {
    RadioProtocolWrapper message;
    RadioProtocolCommand* command = message.add_command();

    joystick->MakeCommand(robot_id, command);

    if (!udp_server.SendProtobuf(message)) {
      cerr << "Send output error" << endl;
    }
    time_helper.Sleep();
  }

  joystick->Close();
}

