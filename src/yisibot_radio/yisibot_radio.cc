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

#include "yisibot_radio/yisibot_radio.h"

#include <fcntl.h>
#include <sys/timerfd.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include "radio_protocol_wrapper.pb.h"
#include "util/timer.h"
#include "yisibot_radio/crc.h"

namespace yisibot_radio {

const pthread_mutex_t kMutexInit = PTHREAD_MUTEX_INITIALIZER;

YisibotRadio::YisibotRadio() {
  robot_commands_mutex_ = kMutexInit;
  num_commands_sent_ = 0;
  num_commands_ = 0;
  num_commands_next_ = 0;
  shutdown_ = false;
}

bool YisibotRadio::Init(const char* serial_device, unsigned int frequency) {
  if (!serial_.open(serial_device, 115200, O_RDWR | O_NONBLOCK)) {
    std::printf("Unable to open serial port \"%s\".\n", serial_device);
    return false;
  }

  // Transmit starting packets
  YisibotRadioCommandPacket start_packet_1{
      0xff, 0xb0, 0x01, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31};

  PrintPacket(start_packet_1);

  serial_.write(&start_packet_1, 25);

  Sleep(1.0 / 60.0);

  YisibotRadioCommandPacket start_packet_2;
  start_packet_2[0] = 0xff;
  start_packet_2[1] = 0xb0;
  start_packet_2[2] = 0x04;
  start_packet_2[3] = 0x05;
  start_packet_2[4] = 0x06;
  start_packet_2[5] = 0x10 + frequency;

  for (unsigned int i = 6; i < 24; i++) {
    start_packet_2[i] = 0x00;
  }

  start_packet_2[24] = CCrc8::calc(start_packet_2, 24);
  PrintPacket(start_packet_2);
  serial_.write(&start_packet_2, 25);

  Sleep(1.0);

  // Start a thread for radio transmission.
  if (pthread_create(&transmit_thread_, NULL, TransmitThread,
                     reinterpret_cast<void*>(this)) != 0) {
    return false;
  }

  return true;
}

void YisibotRadio::Close() {
  shutdown_ = true;
  pthread_join(transmit_thread_, NULL);
}

bool YisibotRadio::Send(const RadioProtocolWrapper& message) {
  // Lock robot commands
  if (pthread_mutex_lock(&robot_commands_mutex_) != 0) return false;

  //     std::cout << "Recieved message with " << message.command_size();
  //     std::cout << " commands " << std::endl;

  for (unsigned int i = 0;
       i < static_cast<unsigned int>(message.command_size()); ++i) {
    bool found_id = false;
    for (unsigned int j = 0; j < num_commands_next_; ++j) {
      if (robot_commands_next_[j].id == message.command(i).robot_id()) {
        Set(message.command(i), &(robot_commands_next_[j]));
        found_id = true;
        break;
      }
    }
    if (!found_id) {
      Set(message.command(i), &(robot_commands_next_[num_commands_next_]));
      num_commands_next_++;
    }
  }

  // num_commands_next_ = static_cast<unsigned int>(message.command_size());
  // for (unsigned int i = 0; i < num_commands_next_; ++i) {
  //  Set(message.command(i), &(robot_commands_next_[i]));
  // }

  // Unlock robot commands
  if (pthread_mutex_unlock(&robot_commands_mutex_) != 0) return false;

  return true;
}

void YisibotRadio::PrintPacket(const YisibotRadioCommandPacket& packet) {
  std::cout << "New Packet" << std::endl;
  for (int i = 0; i < 25; i++) {
    std::printf("%i: %u\n", i, packet[i]);
  }
}

void YisibotRadio::Set(const RadioProtocolCommand& command,
                       YisibotRadioRobotCommand* radio_command) {
  radio_command->id = command.robot_id();
  // radio_command->id = idToYisibotIndex(command.robot_id());

  float command_x_velocity = command.velocity_x() * 100.0;
  // Positive Y is right
  float command_y_velocity = -command.velocity_y() * 100.0;
  float command_rotational_velocity = command.velocity_r() * 40;

  int x_velocity =
      static_cast<int>(command_x_velocity >= 0 ? command_x_velocity + 0.5
                                               : command_x_velocity - 0.5);
  int y_velocity =
      static_cast<int>(command_y_velocity >= 0 ? command_y_velocity + 0.5
                                               : command_y_velocity - 0.5);
  int rotational_velocity = -static_cast<int>(
      command_rotational_velocity >= 0 ? command_rotational_velocity + 0.5
                                       : command_rotational_velocity - 0.5);
  if (x_velocity > 511) {
    x_velocity = 511;
  } else if (x_velocity < -511) {
    x_velocity = -511;
  }

  if (y_velocity > 511) {
    y_velocity = 511;
  } else if (y_velocity < -511) {
    y_velocity = -511;
  }

  if (rotational_velocity > 511) {
    rotational_velocity = 511;
  } else if (rotational_velocity < -511) {
    rotational_velocity = -511;
  }

  int dribble_level = 0;
  // int dribble_direction = 0;
  bool dribble = 0;

  if (command.has_dribbler_spin()) {
    float command_dribble = command.dribbler_spin();
    // todo(slane): Convert to use input dribble level
    if (command_dribble > 0) {
      dribble = true;
      // dribble_direction = 0;
      dribble_level = 3;
    } else if (command_dribble < 0) {
      dribble = true;
      // dribble_direction = 1;
      dribble_level = 3;
    }
  }

  bool shoot = false;
  int shoot_mode = 0;
  int shot_power = 0;

  if (command.has_flat_kick() && command.flat_kick() > 0) {
    shoot = true;
    shoot_mode = 0;
  } else if (command.has_chip_kick() && command.chip_kick() > 0) {
    shoot = true;
    shoot_mode = 1;
  }

  if (shoot) {
    shot_power = 62;  // todo(slane): Convert to use input shot powers
  }

  // Set command byte
  radio_command->packet[0] = (shoot_mode << 6);
  radio_command->packet[0] =
      radio_command->packet[0] | (dribble ? (dribble_level << 4) : 0);

  // Set x velocity info
  radio_command->packet[1] =
      ((x_velocity >= 0) ? 0 : 0x80) | (abs(x_velocity) & 0x7f);

  // Set y velocity info
  radio_command->packet[2] =
      ((y_velocity >= 0) ? 0 : 0x80) | (abs(y_velocity) & 0x7f);
  // Set angular velocity info
  radio_command->packet[3] = ((rotational_velocity >= 0) ? 0 : 0x80) |
                             (abs(rotational_velocity) & 0x7f);

  radio_command->packet[4] = ((abs(x_velocity) & 0x180) >> 1) |
                             ((abs(y_velocity) & 0x180) >> 3) |
                             ((abs(rotational_velocity) & 0x180) >> 5);

  radio_command->packet[5] = (shoot ? shot_power : 0) & 0x7f;
}

void* YisibotRadio::TransmitThread(void* radio_ptr) {
  YisibotRadio& radio = *(reinterpret_cast<YisibotRadio*>(radio_ptr));

  YisibotRadioCommandPacket radio_packet;
  radio_packet[0] = 0xff;
  radio_packet[21] = 0x00;
  radio_packet[22] = 0x00;
  radio_packet[23] = 0x00;
  radio_packet[24] = 0x00;

  int current_command_indices[kRadioCommandsPerYisibotPacket];

  RateLoop time_helper(60);

  while (!radio.shutdown_) {
    // Get the ids to populate the ID byte

    int num_robots_in_packet = 0;

    for (unsigned int i = 0; i < kRadioCommandsPerYisibotPacket &&
                             radio.num_commands_sent_ < radio.num_commands_;
         ++radio.num_commands_sent_, ++i) {
      current_command_indices[i] = radio.num_commands_sent_;
      num_robots_in_packet++;
    }

    // Fill the rest of the packet with null commands
    for (unsigned int i = num_robots_in_packet;
         i < kRadioCommandsPerYisibotPacket; ++i) {
      current_command_indices[i] = -1;
    }

    // Order robots to publish by ID (descending)
    for (int i = 0; i < num_robots_in_packet - 1; i++) {
      int max = i;
      for (int j = i + 1; j < num_robots_in_packet; j++) {
        if (radio.robot_commands_current_[current_command_indices[j]].id <
            radio.robot_commands_current_[current_command_indices[max]].id) {
          max = j;
        }

        if (max != i) {
          int temp = current_command_indices[i];
          current_command_indices[i] = current_command_indices[max];
          current_command_indices[max] = temp;
        }
      }
    }

    // Populate the ID byte
    radio_packet[1] = 0x00;
    radio_packet[2] = 0x00;

    // For each robot command, populate the relevant bytes of the packet
    for (unsigned int i = 0; i < kRadioCommandsPerYisibotPacket; i++) {
      if (current_command_indices[i] != -1) {
        int current_index = current_command_indices[i];
        YisibotRadioRobotCommand current_command =
            radio.robot_commands_current_[current_index];

        if (current_command.id > 7) {
          radio_packet[1] = radio_packet[1] | 1 << (current_command.id - 8);
        } else {
          radio_packet[2] = radio_packet[2] | 1 << current_command.id;
        }

        radio_packet[4 * i + 3] = current_command.packet[0];
        radio_packet[4 * i + 4] = current_command.packet[1];
        radio_packet[4 * i + 5] = current_command.packet[2];
        radio_packet[4 * i + 6] = current_command.packet[3];
        radio_packet[i + 15] = current_command.packet[4];
        radio_packet[i + 18] = current_command.packet[5];
      } else {
        radio_packet[4 * i + 3] = 0x00;
        radio_packet[4 * i + 4] = 0x00;
        radio_packet[4 * i + 5] = 0x00;
        radio_packet[4 * i + 6] = 0x00;
        radio_packet[i + 15] = 0x00;
        radio_packet[i + 18] = 0x00;
      }
    }

    // Print the packet
    radio.PrintPacket(radio_packet);

    // Send the packet
    radio.serial_.write(&radio_packet, sizeof(radio_packet));

    // If all commands have been sent:
    if (radio.num_commands_sent_ == radio.num_commands_) {
      // Lock mutex on robot_commands_next.
      pthread_mutex_lock(&(radio.robot_commands_mutex_));
      // Copy over robot_commands_next to robot_commands_current.
      radio.num_commands_ = radio.num_commands_next_;
      for (unsigned int i = 0; i < radio.num_commands_next_; ++i) {
        radio.robot_commands_current_[i] = radio.robot_commands_next_[i];
      }
      // Reset num_commands_sent and num_commands_next.
      radio.num_commands_sent_ = 0;
      radio.num_commands_next_ = 0;

      // Unlock mutex on robot_commands_next.
      pthread_mutex_unlock(&(radio.robot_commands_mutex_));
    }

    time_helper.Sleep();
  }

  return NULL;
}
}  // namespace yisibot_radio
