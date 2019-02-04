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

#include <pthread.h>
#include <vector>
#include "radio_protocol_wrapper.pb.h"
#include "src/yisibot_radio/serial.h"

#ifndef SRC_YISIBOT_RADIO_YISIBOT_RADIO_H_
#define SRC_YISIBOT_RADIO_YISIBOT_RADIO_H_

namespace yisibot_radio {
typedef unsigned char YisibotRadioCommandPacket[25];

static const int kRadioCommandsPerYisibotPacket = 3;
static const int kMaxRobots = 12;
static const int kFrequency = 0;

// May be deprecated
struct YisibotRadioRobotCommand {
  // The bytes that will be send with the command packet corresponding to this
  // robot's commands
  // In this order:
  // Misc commands (dribble, shot type, dribble power)
  // velocity_x (Sign and lower 7)
  // velocity_y (Sign and lower 7)
  // velocity_rotate (Sign and lower 7)
  // Upper to bits of each velocity, Vx, Vy, Vr
  // Shot power
  char packet[6];

  // Robot ID
  // Between 0 and 11
  int id;
};

class YisibotRadio {
 public:
  // Default constructor
  YisibotRadio();

  // Open serial ports.
  bool Init(const char* serial_device, unsigned int frequency);

  // Close serial ports.
  void Close();

  // Send a set of radio commands.
  bool Send(const RadioProtocolWrapper& messsage);

  // Print the packet to std out
  void PrintPacket(const YisibotRadioCommandPacket& packet);

  // Print the command to std out
  void PrintRobotCommand(const RadioProtocolCommand& packet);

 protected:
  // Translate the RobotCommand packet @cmd into a RadioRobotCommand @radio_cmd.
  // Returns the actuall command sent in @sent_command, accounting for the wheel
  // velocity command saturation and clamping.
  void Set(const RadioProtocolCommand& command,
           YisibotRadioRobotCommand* radio_command);

  int idToYisibotIndex(int robot_id);

 private:
  // Thread function from where all the radio transmissions are made.
  static void* TransmitThread(void* radio_pointer);

  // The current set of robot commands that are in the process of being sent.
  // It is necessary to keep a copy of this since it may take multiple radio
  // transmissions to send out all the commands;
  YisibotRadioRobotCommand robot_commands_current_[kMaxRobots];

  // Keeps track of the number of robot commands that have been sent so far
  // from robot_commands_current.
  unsigned int num_commands_sent_;

  // The number of robots that are to be commanded, and hence the number of
  // valid entries in @robot_commands_next.
  unsigned int num_commands_;

  // The next set of robot commands that will be sent out as soon as the
  // current set has been sent.
  YisibotRadioRobotCommand robot_commands_next_[kMaxRobots];

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
};
}  // namespace yisibot_radio

#endif  // SRC_YISIBOT_RADIO_YISIBOT_RADIO_H_
