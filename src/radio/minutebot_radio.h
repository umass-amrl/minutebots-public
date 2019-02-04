// Copyright 2017 - 2018 slane@cs.umass.edu, jaholtz@umass.edu
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

#include <pthread.h>
#include <vector>
#include "shared/common_includes.h"
#include "yisibot_radio/crc.h"
#include "src/yisibot_radio/serial.h"
#include "radio_protocol_wrapper.pb.h"
#include "motion_control/kinematic_model.h"
#include "radio/kick_curve.h"

#ifndef SRC_RADIO_MINUTEBOT_RADIO_H_
#define SRC_RADIO_MINUTEBOT_RADIO_H_

namespace radio {
typedef unsigned char MinutebotRadioCommandPacket[25];

static const int kMaxRobotsPerPacket = 12;
static const int kMaxRobots = 12;
static const int kFrequency = 0;

struct __attribute__((packed)) RobotCommandPacket {
  // 8 bit Flags, from 0(LSB) to 7(MSB):
  // bit 0-3: Robot ID
  // bit 4: Flat Kick
  // bit 5: Chip Kick
  // bit 6: Send Feedback
  // bit 7: Beep
  uint8_t flags;

  // Velocity of individual wheels, with index 0 corresponding to the front left
  // wheel incrementing clockwise.
  int8_t wheel_velocity[4];

  // Kicker power.
  uint8_t kick_power;

  // Dribbler power.
  int8_t dribble_power;

  // Clear all command flags and values, preserving the robot ID.
  void Clear() {
    flags &= 0x0F;
    for (int i = 0; i < 4; ++i) {
      wheel_velocity[i] = 0;
    }
    kick_power = 0;
    dribble_power = 0;
  }

  // Issue a kick command.
  void Kick(int8_t power, bool chip_kick) {
    if (chip_kick) {
      flags |= 0x20;
    } else {
      flags |= 0x10;
    }
    kick_power = power;
  }

  static RobotCommandPacket IdleCommand() {
    RobotCommandPacket packet;
    packet.flags = 0xF;
    for (size_t i = 0; i < 4; ++i) {
      packet.wheel_velocity[i] = 0;
    }
    packet.kick_power = 0;
    packet.dribble_power = 0;
    return packet;
  }
};  // 7 bytes

struct __attribute__((packed)) RadioCommandPacket {
  // Prelude to radio packet, should be "RP" for "Robot Packet".
  uint8_t prelude[2];

  // RobotPacket data for up to 12 robots.
  RobotCommandPacket robots[kMaxRobots];

  // CRC16 checksum.
  uint8_t fcs[2];

  // Default constructor: fill up prelude, no other initialization.
  RadioCommandPacket() {
    prelude[0] = 'R';
    prelude[1] = 'P';
    for (size_t i = 0; i < kMaxRobots; ++i) {
      robots[i] = RobotCommandPacket::IdleCommand();
    }
    fcs[0] = 0;
    fcs[1] = 0;
  }

  // Copmute and fill up the CRC16 checksum.
  void ComputeCRC() {
    const uint16_t checksum = CCrc16::calc(
      reinterpret_cast<unsigned char*>(this), sizeof(RadioCommandPacket) - 2);
    fcs[0] = (checksum & 0xFF);
    fcs[1] = ((checksum >> 8) & 0xFF);
  }
};  // 2 + 7 * 12 + 2 = 88 bytes

class MinutebotRadio {
 public:
  // Default constructor
  MinutebotRadio();

  // Open serial ports.
  bool Init(const char *serial_device, unsigned int frequency);

  // Close serial ports.
  void Close();

  // Send a set of radio commands.
  bool Send(const RadioProtocolWrapper& messsage);

  // Print the command to std out
  void PrintRobotCommand(const RadioProtocolCommand& packet);

 protected:
  // Translate the RobotCommand packet @cmd into a RadioRobotCommand @radio_cmd.
  // Returns the actuall command sent in @sent_command, accounting for the wheel
  // velocity command saturation and clamping.
  void Set(const  RadioProtocolCommand& command,
           RobotCommandPacket* radio_command);

  int idToYisibotIndex(int robot_id);

 private:
  // Thread function from where all the radio transmissions are made.
  static void* TransmitThread(void* radio_pointer);

  // The current set of robot commands that are in the process of being sent.
  // It is necessary to keep a copy of this since it may take multiple radio
  // transmissions to send out all the commands;
  RobotCommandPacket robot_commands_current_[kMaxRobots];

  // Keeps track of the number of robot commands that have been sent so far
  // from robot_commands_current.
  unsigned int num_commands_sent_;

  // The number of robots that are to be commanded, and hence the number of
  // valid entries in @robot_commands_next.
  unsigned int num_commands_;

  // The next set of robot commands that will be sent out as soon as the
  // current set has been sent.
  RobotCommandPacket robot_commands_next_[kMaxRobots];

  // The number of robots that are to be commanded in the next set, and hence
  // the number of valid entries in @robot_commands_next.
  unsigned int num_commands_next_;

  // The estimated timestamp when the next set of commands will be sent over
  // the air.
  double next_transmit_time_;

  // Mutex lock for @robot_commands_next, @num_commands_next and
  // @next_transmit_time.
  pthread_mutex_t robot_commands_mutex_;

  // The serial device used to send out commands to the robots
  Serial serial_;

  // Used to notify all threads to shut down.
  bool shutdown_;

  // Thread handle for transmit thread.
  pthread_t transmit_thread_;

  // Kinematic model instance to be used for reshaping the commanded wheel
  // velocities
  motion::KinematicModel kinematic_model_;

  std::array<KickCurve, kNumSSLVisionIds> kick_curves_;
};
}  // namespace radio

#endif  // SRC_RADIO_MINUTEBOT_RADIO_H_
