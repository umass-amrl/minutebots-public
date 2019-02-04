// Copyright 2011-2017 jaholtz@cs.umass.edu
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


#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <cstdarg>
#include <iostream>
#include "util/timer.h"
#include "yisibot_radio/crc.h"
#include "yisibot_radio/serial.h"


#ifndef SRC_RADIO_RADIO_HELPER_H_
#define SRC_RADIO_RADIO_HELPER_H_

namespace radio_helper {

enum ParseState { start, start_found, full_length};

  union DoubleArray {
    double value;
    char buffer[sizeof(double)];
  };

  union Int32Array {
    uint32_t value;
    char buffer[sizeof(uint32_t)];
  };

  // static const int kPayloadSize = 60;
  static const int kPayloadSize =
      6 * 6 - sizeof(Int32Array) - sizeof(DoubleArray);

  struct __attribute__((packed)) RadioPacket {
    char header[2];
    DoubleArray time;
    Int32Array count;
    char payload[kPayloadSize];
    char checksum[2];

    RadioPacket() {
      header[0] = 0;
      header[1] = 0;
      time.value = 0;
      count.value = 0;
      checksum[0] = 0;
      checksum[1] = 1;
    }
  };

  static const int kRadioPacketSize = sizeof(RadioPacket);


  void PrintBufferInHex(unsigned char* buffer, int size);

  // Parses a RadioPacket from the network.
  // Inputs::
  //    receive_buffer - the newly received data from network
  //    receive_size - the number of bytes received
  //    partial_packet_buffer - Storage buffer for partially received packets.
  //        The start of the buffer is guaranteed to be a valid start of a
  //        packet.
  //    partial_packet_buffer_size -
  //    state - a pointer to the current ParseState of the parser.
  //    buffer_index - pointer to index to the saved buffer
  //    packet  - the parsed packet that will be filled iff a complete packet
  //              has been received.
  int ParseRadioPacket(const char* receive_buffer,
                       int receive_size,
                       unsigned char* partial_packet_buffer,
                       int* partial_packet_buffer_size,
                       ParseState* parse_state,
                       RadioPacket* parsed_packet);

}  // namespace radio_helper

#endif  // SRC_RADIO_RADIO_HELPER_H_

