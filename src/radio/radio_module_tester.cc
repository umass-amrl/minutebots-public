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
#include "yisibot_radio/crc.h"
#include "yisibot_radio/serial.h"
#include "radio/radio_helper.h"


using std::string;
using radio_helper::DoubleArray;
using radio_helper::RadioPacket;
using radio_helper::kRadioPacketSize;
using radio_helper::ParseRadioPacket;
using radio_helper::PrintBufferInHex;
Serial serial_port_;

const bool kDebug = false;
// Client mode infinite loop. Waits to receive packets, prints latency
// information.
void ReceiveData() {
  ScopedFile fid("latency.txt", "w", true);
  static const uint32_t kBufferSize = sizeof(RadioPacket);
  static const uint32_t kSavedSize = 3 *sizeof(RadioPacket);
  printf("Client mode\n");
  char receive_buffer[kBufferSize];
  unsigned char saved[kSavedSize];
  radio_helper::ParseState state = radio_helper::ParseState::start;
  int saved_size = 0;
  while (true) {
    RadioPacket packet;
    if (serial_port_.waitForInput(0)) {
      int size = serial_port_.read(receive_buffer, kBufferSize);
      const double start_time = GetWallTime();
      if (kDebug) {
        std::cout << "Read Packet" << std::endl;
        PrintBufferInHex(reinterpret_cast<unsigned char*>(&receive_buffer),
                          size);
      }
      int status = ParseRadioPacket(
                                  reinterpret_cast<const char*>(receive_buffer),
                                  size,
                                  saved,
                                  &saved_size,
                                  &state,
                                  &packet);
      if (status == 1) {
        const uint32_t& packet_counter = packet.count.value;
        const double& packet_time = packet.time.value;
        const double cur_time = GetWallTime();
        // Prints to check progress without too much overhead.

        if (packet_counter % 100 == 0) {
          printf("Latency: %f \n", cur_time - packet_time);
        }
        // Writing latency information to file.
        fprintf(fid,
                "%" PRIu32 " %.6f %.6f %.6f\n",
                packet_counter,
                packet_time,
                cur_time,
                cur_time - start_time);
      }
    }
  }
}

// Server mode infinite loop. Continuously sends packets at a fixed transmit
// rate.
void ServerMode() {
  static const float kTransmitFrequency = 62.503125;
  // static const float kTransmitFrequency = 2.0 * 62.503125;
  printf("Server mode\n");\
  CCrc16 crc;
  RateLoop loop(kTransmitFrequency);
  RadioPacket packet;
  packet.header[0] = 'R';
  packet.header[1] = 'P';
  memset(packet.payload, 'a', radio_helper::kPayloadSize);
  serial_port_.write(&packet, sizeof(packet));
  Sleep(0.2);
  int count = 0;
  while (true) {
    loop.Sleep();
    packet.time.value = GetWallTime();
    if (kDebug) {
      printf("Time: %.6f\n", packet.time.value);
    }
    packet.count.value = count;
    const uint16_t checksum = crc.calc(
      reinterpret_cast<unsigned char*>(&packet), kRadioPacketSize - 2);
    packet.checksum[0] = (checksum & 0xFF);
    packet.checksum[1] = ((checksum >> 8) & 0xFF);
    if (kDebug) {
      printf("Checksum: %d, c1: %d, c2: %d \n", checksum, checksum & 0xFF,
            ((checksum >> 8) & 0xFF));
    }
    // Write to serial_port
    if (kDebug) {
      PrintBufferInHex(reinterpret_cast<unsigned char*>(&packet),
                       sizeof(packet));
    }
    const int bytes_written = serial_port_.write(&packet, sizeof(packet));
    if (bytes_written != sizeof(packet)) {
      perror("Error writing to serial port");
    }
    count++;
    // Prints to track progress without too much overhead.
    if (count % 100 == 0) {
      std::cout << count << std::endl;
    }
  }
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    fprintf(stderr, "ERROR: Must specify serial port.\n");
    return 1;
  }
  const char* serial_port_name = argv[1];
  if (!serial_port_.open(serial_port_name, 460800)) {
    fprintf(stderr, "ERROR: Unable to open serial port.\n");
    exit(2);
  }

  if (argc > 2 && strncmp("-c", argv[2], 2) == 0) {
    ReceiveData();
  } else {
    ServerMode();
  }
  return 0;
}
