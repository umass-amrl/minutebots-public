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

#include "radio/radio_helper.h"

using std::string;

namespace radio_helper {

  void PrintBufferInHex(unsigned char* buffer, int size) {
    printf("Size: %d\n", size);
    for (int i = 0; i < size; i++) {
      printf("%X", buffer[i]);
    }
    std::cout << std::endl;
    for (int i = 0; i < size; i++) {
      std::cout << buffer[i];
    }
    std::cout << std::endl;
  }

// Parses received data, and fills out the packet pointer if succesful. This
// function needs to be C99 compatible so as to be usable on the robot
// microcontrollers as well. The returned value is 1 iff the packet was parsed
// correctly, 0 otherwise.
int ParseRadioPacket(const char* receive_buffer,
                     int receive_size,
                     unsigned char* partial_packet_buffer,
                     int* partial_packet_buffer_size,
                     ParseState* parse_state,
                     RadioPacket* parsed_packet) {
  static const bool kDebug = false;
  static const char kFirstHeader = 'R';
  static const char kSecondHeader = 'P';
  CCrc16 crc;
  if (kDebug) {
    std::cout << "Starting Parser" << std::endl;
  }
  ParseState& state = *parse_state;
  int& partial_buffer_size = *partial_packet_buffer_size;
  int partial_buffer_index = 0;
  // Copy received bytes to the end of the partial packet buffer.
  memcpy(partial_packet_buffer + *partial_packet_buffer_size, receive_buffer,
      receive_size);
  partial_buffer_size += receive_size;
  if (kDebug) {
    PrintBufferInHex(partial_packet_buffer, partial_buffer_size);
  }
  bool try_for_packet = true;
  while (try_for_packet) {
    try_for_packet = false;
    // Find the start of the packet. It will be marked by the characters 'R',
    // 'P'. Check successive pairs of bytes, looking for an RP
    while (partial_buffer_index < (partial_buffer_size - 1) && state == start) {
      char current_byte(partial_packet_buffer[partial_buffer_index]);
      char next_byte = partial_packet_buffer[partial_buffer_index + 1];
      // If an RP was found, remove all bytes preceding the RP and change
      // state to START_FOUND
      if (current_byte == kFirstHeader && next_byte == kSecondHeader) {
        if (kDebug) {
          std::cout << "found start" << std::endl;
          std::cout << "Index: " << partial_buffer_index << std::endl;
        }
        // remove all bytes preceeding the RP
        if (partial_buffer_index > 0) {
          const int kAdjustedSize = partial_buffer_size - partial_buffer_index;
          char temp[kAdjustedSize];
          // Copying to temp to avoid copying from partial_buffer to
          // partial_buffer
          memcpy(temp,
                 partial_packet_buffer + partial_buffer_index,
                 partial_buffer_size - partial_buffer_index);
          partial_buffer_size = partial_buffer_size
              - partial_buffer_index;
          memcpy(partial_packet_buffer, temp, partial_buffer_size);
          // Set index back to beginning
          partial_buffer_index = 0;
        }
        // Change state to start found
        state = start_found;
      } else {
        // Update index
        partial_buffer_index++;
      }
    }
    // If we have searched the partial buffer and not found a packet start.
    if (state == start) {
      // Remove all but the last character from the partial packet buffer
      // it could still be an R.
      char temp[1];
      // Copying to temp to avoid copying from partial_buffer to
      // partial_buffer
      memcpy(temp,
             partial_packet_buffer + (partial_buffer_size - 2),
             1);
      partial_buffer_size = 1;
      memcpy(partial_packet_buffer, temp, partial_buffer_size - 1);
      // Set index back to beginning
      partial_buffer_index = 0;
    }
    //  At this point, there is guaranteed to be at least two bytes of a partial
    //  packet in the partial packet buffer.

    // If enough data has been received:
    if (state == start_found && partial_buffer_size >= kRadioPacketSize) {
      if (kDebug) {
        std::cout << "Parsing" << std::endl;
        PrintBufferInHex(partial_packet_buffer, partial_buffer_size);
      }
      // Read in checksums from message, and calculate checksums from packet
      const uint16_t checksum = crc.calc(partial_packet_buffer,
                                        kRadioPacketSize - 2);
      const char read_check1 = partial_packet_buffer[kRadioPacketSize - 2];
      const char read_check2 = partial_packet_buffer[kRadioPacketSize - 1];
      const char actual_check1 = (checksum & 0xFF);
      const char actual_check2 = ((checksum >> 8) & 0xFF);
      if (kDebug) {
        printf("Checksum: %d, c1: %d, c2: %d \n", checksum, actual_check1,
               actual_check2);
      }
      // If checksum passes:
      if (read_check1 == actual_check1 && read_check2 == actual_check2) {
        if (kDebug) {
          std::cout << "Matched Checks" << std::endl;
        }
        // Copy over into parsed_packet
        memcpy(parsed_packet, partial_packet_buffer, sizeof(RadioPacket));

        // Remove the packets data from the partial_packet_buffer
        const int kAdjustedSize = partial_buffer_size
            - sizeof(RadioPacket);
        if (kDebug) {
          std::cout << "Copied to Packet" << std::endl;
        }
        if (kAdjustedSize > 0) {
          char temp[kAdjustedSize];
          // Copying to temp to avoid copying from partial_buffer to
          // partial_buffer
          memcpy(temp,
                 partial_packet_buffer + partial_buffer_index,
                 kAdjustedSize);
          memcpy(partial_packet_buffer, temp, kAdjustedSize);
          // Begin searching for a start state
          state = start;
        }
        partial_buffer_size = kAdjustedSize;
        // return 1 to indicate that the packet was parsed
        // Note that if there is any partial packet received after this
        // packet, it will be untouched until more data is received.
        return 1;
      } else {
        // This means that a corrupt packet was received.
        // Remove only start 2 bytes, and go to FIND_START state.
        const int kAdjustedSize = partial_buffer_size - 2;
        char temp[kAdjustedSize];
        // Copying to temp to avoid copying from partial_buffer to
        // partial_buffer
        partial_buffer_size = kAdjustedSize;
        memcpy(temp,
               partial_packet_buffer + 2,
               partial_buffer_size);
        memcpy(partial_packet_buffer, temp, partial_buffer_size);
        // Need to go back to searching for a new packet start
        try_for_packet = true;
        state = start;
      }

    } else {
      // Return 0 to indicate that a packet was not parsed
      return 0;
    }
  }
  // Return 0 to indicate that a packet was not parsed
  return 0;
}

}  // namespace radio_helper

