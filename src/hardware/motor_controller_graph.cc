// Copyright 2017 joydeepb@cs.umass.edu
//
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Radio module performance tester for UMass MinuteBots.
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

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <iostream>
#include <cinttypes>
#include <fstream>

#include "shared/common_includes.h"
#include "util/timer.h"
#include "yisibot_radio/serial.h"


using std::string;
Serial serial_port_;

const bool kDebug = false;

struct DebugPacket {
  int16_t p;
  int16_t i;
  int16_t d;
  int16_t pwm;
};

union Int16{
  char data[2];
  int16_t value;
};

int16_t UnpackInt(char* buffer) {
  Int16 x;
  x.data[0] = buffer[0];
  x.data[1] = buffer[1];
  return (x.value);
}

bool ParsePacket(char* buffer, int len, DebugPacket* pkt) {
  static char partial_packet[12];
  static int partial_packet_size = 0;
  bool packet_found = false;
  for (int i = 0; i < len; ++i) {
    partial_packet[partial_packet_size] = buffer[i];
    partial_packet_size++;
    if (partial_packet_size == 12) {
      if (partial_packet[0] == 'D' && partial_packet[1] == 'D') {
        if (kDebug) printf("Packet start found\n");
        int16_t checksum = 0;
        for (int j = 0; j < 10; ++j) {
          checksum += partial_packet[j];
        }
        const int16_t received_checksum = UnpackInt(partial_packet + 10);
        if (kDebug) {
          printf("Checksum: %d received: %d\n",
                  checksum,
                  received_checksum);
        }
        if (true || checksum == received_checksum) {
          if (kDebug) printf("Valid packet\n");
          packet_found = true;
          pkt->p = UnpackInt(partial_packet + 2);
          pkt->i = UnpackInt(partial_packet + 4);
          pkt->d = UnpackInt(partial_packet + 6);
          pkt->pwm = UnpackInt(partial_packet + 8);
          partial_packet_size = 0;
        } else {
          // Discard the oldest byte.
          for (int j = 0; j + 1 < 12; ++j) {
            partial_packet[j] = partial_packet[j + 1];
          }
          partial_packet_size--;
        }
      } else {
        // Discard the oldest byte.
        for (int j = 0; j + 1 < 12; ++j) {
          partial_packet[j] = partial_packet[j + 1];
        }
        partial_packet_size--;
      }
    }
  }
  return packet_found;
}

// Client mode infinite loop. Waits to receive packets, prints latency
// information.
void ReceiveData() {
  ScopedFile fid("data.txt", "w", true);
  static const uint32_t kBufferSize = 128;
  char receive_buffer[kBufferSize];
  DebugPacket packet;
  while (true) {
    if (serial_port_.waitForInput(0)) {
      int size = serial_port_.read(receive_buffer, kBufferSize);
      receive_buffer[size] = 0;
      if (ParsePacket(receive_buffer, size, &packet)) {
        printf("Received packet %5d %5d %5d %5d\n",
               packet.p,
               packet.i,
               packet.d,
               packet.pwm);
        fprintf(fid,
                "%5d,%5d,%5d,%5d\n",
                packet.p,
                packet.i,
                packet.d,
                packet.pwm);
        fflush(fid);
      }
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    fprintf(stderr, "ERROR: Must specify serial port.\n");
    return 1;
  }
  const char* serial_port_name = argv[1];
  if (!serial_port_.open(serial_port_name, 230400)) {
    fprintf(stderr, "ERROR: Unable to open serial port.\n");
    exit(2);
  }

  ReceiveData();
  return 0;
}
