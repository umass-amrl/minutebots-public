// Copyright 2016 - 2017 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// UDP Networking Library Tester.
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

#include <inttypes.h>
#include <stdio.h>

#include <glog/logging.h>
#include <iostream>
#include <string>

#include "net/netraw.h"
#include "util/timer.h"
#include "netraw_test_message.pb.h"
#include "radio_protocol_wrapper.pb.h"

using net::UDPMulticastServer;
using std::cerr;
using std::cout;
using std::endl;
using std::string;


// UDP server instance to use for sending or receiving test data.
UDPMulticastServer udp_server_;

void SetupVelocityCommands(RadioProtocolWrapper* message) {
  message->clear_command();
  for (int robot_index = 0; robot_index < 6; ++robot_index) {
    RadioProtocolCommand* data = message->add_command();
    data->set_robot_id(robot_index);
    data->set_velocity_x(1);  // Drive forward at 1 m/s.
    data->set_velocity_y(0);
    data->set_velocity_r(0);
    data->set_flat_kick(0);
    data->set_chip_kick(0);
    data->set_dribbler_spin(-1);  // Add max backspin.
  }
}

void ReceiveLoop() {
  RadioProtocolWrapper message;
  for (int i = 0;
       udp_server_.ReceiveProtobuf(&message);
       ++i) {
    LOG(INFO) << message.command_size() << "\n";
  }
}

void TransmitLoop() {
  static const double kTransmitRate = 120.0;
  RadioProtocolWrapper message;
  RateLoop loop(kTransmitRate);
  SetupVelocityCommands(&message);
  while (true) {
    if (!udp_server_.SendProtobuf(message)) {
      printf("Send error\n");
    }
    loop.Sleep();
  }
}


int main(int num_arguments, char** arguments) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InitGoogleLogging(arguments[0]);
  FLAGS_stderrthreshold = 0;  // INFO level logging.
  FLAGS_logtostderr = true;  // Don't log to disk.
  static const string kUDPAddress = "227.9.7.3";
  static const int kUDPPort = 64324;

  bool client_mode = false;
  if (!udp_server_.Open(kUDPAddress, kUDPPort, client_mode)) {
    cerr << "Error opening UDP port, exiting.\n";
    return -1;
  }
  CHECK(udp_server_.IsOpen());
  if (num_arguments > 1 && string(arguments[1]) == string("-c")) {
    client_mode = true;
    cout << "Client mode\n";
  } else {
    cout << "Server mode\n";
  }

  if (client_mode) {
    ReceiveLoop();
  } else {
    TransmitLoop();
  }

  udp_server_.Close();
  return 0;
}
