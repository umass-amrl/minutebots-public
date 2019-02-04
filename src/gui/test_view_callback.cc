// Copyright 2018 joydeepb@cs.umass.edu
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

#include "shared/common_includes.h"
#include "net/netraw.h"
#include "util/timer.h"
#include "viewer_callback.pb.h"

using net::UDPMulticastServer;

const char* TeamName(const MinuteBotsProto::SelectedRobot_Team& team) {
  if (team == MinuteBotsProto::SelectedRobot_Team_TEAM_BLUE) {
    return "Blue";
  } else if (team == MinuteBotsProto::SelectedRobot_Team_TEAM_YELLOW) {
    return "Yellow";
  }
  return "Unknown team";
}

int main(int num_arguments, char** arguments) {
  // UDP server instance to use for sending or receiving test data.
  UDPMulticastServer udp_server;
  CHECK(udp_server.Open(DATA_STREAM_VIEWER_IP, DATA_STREAM_VIEWER_PORT, true));
  CHECK(udp_server.IsOpen());

  MinuteBotsProto::MouseCallback msg;
  MinuteBotsProto::MouseEvent event;
  while (udp_server.ReceiveProtobuf(&msg)) {
    if (msg.has_mouse_down()) {
      printf("Mouse Down ");
      event = msg.mouse_down();
    } else if (msg.has_mouse_up()) {
      printf("Mouse Up ");
      event = msg.mouse_up();
    } else if (msg.has_mouse_move()) {
      printf("Mouse Move ");
      event = msg.mouse_move();
    }
    printf(" button:%d loc:%8.1f,%8.1f Ctrl:%d Alt:%d SHIFT:%d  Selected: ",
           event.button(),
           event.x(),
           event.y(),
           event.ctrl(),
           event.alt(),
           event.shift());
    if (event.selection() ==
        MinuteBotsProto::MouseEvent_SelectionType_BALL_SELECTED) {
      printf("Ball\n");
    } else if (event.selection() ==
        MinuteBotsProto::MouseEvent_SelectionType_ROBOT_SELECTED) {
      printf("%s Robot %X\n",
             TeamName(event.selected_robot().team()),
             event.selected_robot().id());
    }
  }

  udp_server.Close();
  return 0;
}
