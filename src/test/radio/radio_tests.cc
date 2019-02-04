// Copyright 2017 jaholtz@cs.umass.edu
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

#include <stddef.h>
#include "glog/logging.h"
#include "radio/radio_helper.h"
#include "shared/common_includes.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

using radio_helper::RadioPacket;
using radio_helper::DoubleArray;
using radio_helper::Int32Array;
using radio_helper::kRadioPacketSize;
using radio_helper::kPayloadSize;

TEST(RadioTest, PacketPacking) {
  int offset = offsetof(RadioPacket, header);
  int actual_offset = 0;
  ASSERT_EQ(offset, actual_offset);
  offset = offsetof(RadioPacket, time);
  actual_offset += 2;
  ASSERT_EQ(offset, actual_offset);
  offset = offsetof(RadioPacket, count);
  actual_offset += 8;
  ASSERT_EQ(offset, actual_offset);
  offset = offsetof(RadioPacket, payload);
  actual_offset += 4;
  ASSERT_EQ(offset, actual_offset);
  offset = offsetof(RadioPacket, checksum);
  actual_offset += kPayloadSize;
  ASSERT_EQ(offset, actual_offset);
}

TEST(RadioTest, ParseGoodPacket) {
  radio_helper::ParseState state = radio_helper::ParseState::start;
  RadioPacket packet;
  packet.header[0] = 'R';
  packet.header[1] = 'P';
  packet.checksum[0] = -98;
  packet.checksum[1] = -4;
  packet.count.value = 0;
  packet.time.value = 5000.0;
  memset(packet.payload, 'a', kPayloadSize);
  RadioPacket parsed;
  unsigned char* temp = reinterpret_cast<unsigned char*>(&packet);
  string in_string;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  static const uint32_t kSavedSize = 3 * sizeof(RadioPacket);
  unsigned char saved[kSavedSize];
  int saved_size = 0;
  int res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize,
                                           saved, &saved_size, &state, &parsed);
  ASSERT_EQ(res, 1);
  string packet_payload(kPayloadSize, ' ');
  memcpy(&packet_payload[0], packet.payload, kPayloadSize);
  string parsed_payload(kPayloadSize, ' ');
  memcpy(&parsed_payload[0], packet.payload, kPayloadSize);
  ASSERT_EQ(strcmp(packet_payload.c_str(), parsed_payload.c_str()), 0);
  ASSERT_EQ(parsed.header[0], 'R');
  ASSERT_EQ(parsed.header[1], 'P');
  ASSERT_EQ(packet.time.value, parsed.time.value);
  ASSERT_EQ(parsed.count.value, packet.count.value);
  in_string.clear();
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 1,
                                       saved, &saved_size, &state, &parsed);

  memcpy(&packet_payload[0], packet.payload, kPayloadSize);
  memcpy(&parsed_payload[0], packet.payload, kPayloadSize);

  ASSERT_EQ(res, 1);
  ASSERT_EQ(parsed.count.value, packet.count.value);
  ASSERT_EQ(parsed.time.value, packet.time.value);
  ASSERT_EQ(strcmp(packet_payload.c_str(), parsed_payload.c_str()), 0);
  ASSERT_EQ(parsed.header[0], 'R');
  ASSERT_EQ(parsed.header[1], 'P');

  in_string.clear();
  in_string.push_back(' ');
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 2,
                                       saved, &saved_size, &state, &parsed);

  memcpy(&packet_payload[0], packet.payload, kPayloadSize);
  memcpy(&parsed_payload[0], packet.payload, kPayloadSize);
  ASSERT_EQ(res, 1);
  ASSERT_EQ(strcmp(packet_payload.c_str(), parsed_payload.c_str()), 0);
  ASSERT_EQ(parsed.count.value, packet.count.value);
  ASSERT_EQ(parsed.time.value, packet.time.value);
  ASSERT_EQ(parsed.header[0], 'R');
  ASSERT_EQ(parsed.header[1], 'P');

  in_string.clear();
  in_string.push_back(' ');
  in_string.push_back(' ');
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 3,
                                       saved, &saved_size, &state, &parsed);

  memcpy(&packet_payload[0], packet.payload, kPayloadSize);
  memcpy(&parsed_payload[0], packet.payload, kPayloadSize);

  ASSERT_EQ(res, 1);
  ASSERT_EQ(strcmp(packet_payload.c_str(), parsed_payload.c_str()), 0);
  ASSERT_EQ(parsed.count.value, packet.count.value);
  ASSERT_EQ(parsed.time.value, packet.time.value);
  ASSERT_EQ(parsed.header[0], 'R');
  ASSERT_EQ(parsed.header[1], 'P');
}

TEST(RadioTest, ParseBadHeader) {
  radio_helper::ParseState state = radio_helper::ParseState::start;
  RadioPacket packet;
  packet.header[0] = 'J';
  packet.header[1] = 'P';
  packet.checksum[0] = -115;
  packet.checksum[1] = 2;
  packet.count.value = 0;
  memset(packet.payload, 'a', kPayloadSize);
  packet.time.value = 5000.0;
  RadioPacket parsed;
  unsigned char* temp = reinterpret_cast<unsigned char*>(&packet);
  string in_string;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  static const uint32_t kSavedSize = 3 * sizeof(RadioPacket);
  unsigned char saved[kSavedSize];
  int saved_size = 0;
  int res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize,
                                           saved, &saved_size, &state, &parsed);
  ASSERT_EQ(res, 0);
  in_string.clear();
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 1,
                                       saved, &saved_size, &state, &parsed);

  ASSERT_EQ(res, 0);

  in_string.clear();
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 2,
                                       saved, &saved_size, &state, &parsed);
  ASSERT_EQ(res, 0);

  in_string.clear();
  in_string.push_back(' ');
  in_string.push_back(' ');
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 3,
                                       saved, &saved_size, &state, &parsed);

  ASSERT_EQ(res, 0);
}

TEST(RadioTest, ParseBadChecksum) {
  radio_helper::ParseState state = radio_helper::ParseState::start;
  RadioPacket packet;
  packet.header[0] = 'J';
  packet.header[1] = 'P';
  packet.checksum[0] = 0;
  packet.checksum[1] = 0;
  packet.count.value = 0;
  memset(packet.payload, 'a', kPayloadSize);
  packet.time.value = 5000.0;
  RadioPacket parsed;
  unsigned char* temp = reinterpret_cast<unsigned char*>(&packet);
  string in_string;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  static const uint32_t kSavedSize = 3 * sizeof(RadioPacket);
  unsigned char saved[kSavedSize];
  int saved_size = 0;
  int res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize,
                                           saved, &saved_size, &state, &parsed);
  ASSERT_EQ(res, 0);
  in_string.clear();
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 1,
                                       saved, &saved_size, &state, &parsed);

  ASSERT_EQ(res, 0);

  in_string.clear();
  in_string.push_back(' ');
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 2,
                                       saved, &saved_size, &state, &parsed);
  ASSERT_EQ(res, 0);

  in_string.clear();
  in_string.push_back(' ');
  in_string.push_back(' ');
  in_string.push_back(' ');
  state = radio_helper::ParseState::start;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize; ++i) {
    in_string.push_back(temp[i]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize + 3,
                                       saved, &saved_size, &state, &parsed);

  ASSERT_EQ(res, 0);
}

TEST(RadioTest, ParseSplitMessage) {
  radio_helper::ParseState state = radio_helper::ParseState::start;
  RadioPacket packet;
  packet.header[0] = 'R';
  packet.header[1] = 'P';
  packet.checksum[0] = -98;
  packet.checksum[1] = -4;
  packet.count.value = 0;
  memset(packet.payload, 'a', kPayloadSize);
  packet.time.value = 5000.0;
  RadioPacket parsed;
  unsigned char* temp = reinterpret_cast<unsigned char*>(&packet);
  string in_string;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize / 2; ++i) {
    in_string.push_back(temp[i]);
  }
  static const uint32_t kSavedSize = 3 * sizeof(RadioPacket);
  unsigned char saved[kSavedSize];
  int saved_size = 0;
  int res =
      radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize / 2,
                                     saved, &saved_size, &state, &parsed);

  ASSERT_EQ(res, 0);
  in_string.clear();
  for (size_t i = 0; i < radio_helper::kRadioPacketSize / 2; ++i) {
    in_string.push_back(temp[i + (radio_helper::kRadioPacketSize / 2)]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize / 2,
                                       saved, &saved_size, &state, &parsed);

  string packet_payload(kPayloadSize, ' ');
  memcpy(&packet_payload[0], packet.payload, kPayloadSize);
  string parsed_payload(kPayloadSize, ' ');
  memcpy(&parsed_payload[0], packet.payload, kPayloadSize);
  ASSERT_EQ(res, 1);
  ASSERT_EQ(strcmp(packet_payload.c_str(), parsed_payload.c_str()), 0);
  ASSERT_EQ(parsed.count.value, packet.count.value);
  ASSERT_EQ(parsed.time.value, packet.time.value);
  ASSERT_EQ(parsed.header[0], 'R');
  ASSERT_EQ(parsed.header[1], 'P');
}

TEST(RadioTest, NestedHeader) {
  radio_helper::ParseState state = radio_helper::ParseState::start;
  RadioPacket packet;
  packet.header[0] = 'R';
  packet.header[1] = 'P';
  packet.checksum[0] = -98;
  packet.checksum[1] = -4;
  packet.count.value = 0;
  memset(packet.payload, 'a', kPayloadSize);
  packet.time.value = 5000.0;
  RadioPacket parsed;
  unsigned char* temp = reinterpret_cast<unsigned char*>(&packet);
  string in_string;
  for (size_t i = 0; i < radio_helper::kRadioPacketSize / 2; ++i) {
    in_string.push_back(temp[i]);
  }
  for (size_t i = 0; i < radio_helper::kRadioPacketSize / 2; ++i) {
    in_string.push_back(temp[i]);
  }
  static const uint32_t kSavedSize = 3 * sizeof(RadioPacket);
  unsigned char saved[kSavedSize];
  int saved_size = 0;
  int res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize,
                                           saved, &saved_size, &state, &parsed);

  ASSERT_EQ(res, 0);
  in_string.clear();
  for (size_t i = 0; i < radio_helper::kRadioPacketSize / 2; ++i) {
    in_string.push_back(temp[i + (radio_helper::kRadioPacketSize / 2)]);
  }
  res = radio_helper::ParseRadioPacket(in_string.c_str(), kRadioPacketSize / 2,
                                       saved, &saved_size, &state, &parsed);
  string packet_payload(kPayloadSize, ' ');
  memcpy(&packet_payload[0], packet.payload, kPayloadSize);
  string parsed_payload(kPayloadSize, ' ');
  memcpy(&parsed_payload[0], packet.payload, kPayloadSize);
  ASSERT_EQ(res, 1);
  ASSERT_EQ(strcmp(packet_payload.c_str(), parsed_payload.c_str()), 0);
  ASSERT_EQ(parsed.count.value, packet.count.value);
  ASSERT_EQ(parsed.time.value, packet.time.value);
  ASSERT_EQ(parsed.header[0], 'R');
  ASSERT_EQ(parsed.header[1], 'P');
}
