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

using net::UDPMulticastServer;
using std::cerr;
using std::cout;
using std::endl;
using std::string;


// UDP server instance to use for sending or receiving test data.
UDPMulticastServer udp_server_;

void ReceiveLoop() {
  static const double kLowPassConstant = 0.99;
  double mean_latency = 0.0;
  double t_last_receive = 0.0;
  double mean_rate = 0.0;
  NetrawTestMessage message;
  uint32_t last_count = 0;
  uint32_t dropped_packets = 0;
  for (int i = 0;
       udp_server_.ReceiveProtobuf(&message);
       ++i) {
    const double t_now = GetMonotonicTime();
    const double latency = t_now - message.send_time();
    dropped_packets += (message.count() - (last_count + 1));
    last_count = message.count();
    if (i == 0) {
      mean_latency = latency;
    } else {
      mean_latency = kLowPassConstant * mean_latency +
          (1.0 - kLowPassConstant) * latency;
    }
    const double current_rate = 1.0 / (t_now - t_last_receive);
    if (i < 3) {
      mean_rate = current_rate;
    } else {
      mean_rate = kLowPassConstant * mean_rate +
          ((1.0 - kLowPassConstant) * current_rate);
    }
    if (i % 120 == 0) {
      printf("Mean latency: %.3fus rate: %.2fHz dropped:%u\n",
            mean_latency * 1e6,
            mean_rate,
            dropped_packets);
    }
    t_last_receive = t_now;
  }
}

void TransmitLoop() {
  static const double kTransmitRate = 120.0;
  static const int kPayloadSize = 64000;
  NetrawTestMessage message;
  message.mutable_payload()->resize(kPayloadSize, ' ');
  RateLoop loop(kTransmitRate);
  uint32_t count = 0;
  while (true) {
    message.set_send_time(GetMonotonicTime());
    ++count;
    message.set_count(count);
    if (!udp_server_.SendProtobuf(message)) {
      printf("Send error\n");
    }
    loop.Sleep();
  }
}


int main(int num_arguments, char** arguments) {
  static const string kUDPAddress = "127.0.0.1";
  // static const string kUDPAddress = "224.5.10.1";
  int port_number = 10008;

  bool client_mode = false;
  if (num_arguments > 1 && string(arguments[1]) == string("-c")) {
    client_mode = true;
    cout << "Client mode\n";
  } else {
    cout << "Server mode\n";
  }

  if (!udp_server_.Open(kUDPAddress, port_number, client_mode)) {
    cerr << "Error opening UDP port, exiting.\n";
    return -1;
  }
  CHECK(udp_server_.IsOpen());

  if (client_mode) {
    ReceiveLoop();
  } else {
    TransmitLoop();
  }

  udp_server_.Close();
  return 0;
}
