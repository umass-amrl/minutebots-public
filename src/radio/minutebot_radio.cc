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

#include "radio/minutebot_radio.h"

#include <fcntl.h>
#include <sys/timerfd.h>
#include <cstdio>
#include <iostream>
#include <limits>
#include <vector>
#include "radio_protocol_wrapper.pb.h"
#include "util/pthread_utils.h"
#include "util/timer.h"
#include "yisibot_radio/crc.h"

// using std::isnan;
// using std::isinf;

namespace radio {

const pthread_mutex_t kMutexInit = PTHREAD_MUTEX_INITIALIZER;
static constexpr bool kKickTune = false;

MinutebotRadio::MinutebotRadio() : kinematic_model_(motion::KinematicModel()) {
  robot_commands_mutex_ = kMutexInit;
  num_commands_sent_ = 0;
  num_commands_ = 0;
  num_commands_next_ = 0;
  shutdown_ = false;
  for (SSLVisionId i = 0; i < kick_curves_.size(); ++i) {
    kick_curves_[i] = KickCurve(i);
  }
  std::cout << "===================\n";
  if (kKickTune) {
    std::cout << "In kick tune mode.\n";
  } else if (!kRadioUsePassedPower) {
    std::cout << "In hardcoded mode.\n";
  } else {
    std::cout << "In learned curve mode.\n";
  }
  std::cout << "===================\n";
}

bool MinutebotRadio::Init(const char* serial_device, unsigned int frequency) {
  if (!serial_.open(serial_device, 230400)) {
    std::printf("Unable to open serial port \"%s\".\n", serial_device);
    return false;
  }

  // Start a thread for radio transmission.
  return (pthread_create(&transmit_thread_, NULL, TransmitThread,
                         reinterpret_cast<void*>(this)) == 0);
}

void MinutebotRadio::Close() {
  shutdown_ = true;
  pthread_join(transmit_thread_, NULL);
}

bool MinutebotRadio::Send(const RadioProtocolWrapper& message) {
  static const bool kDebug = false;

  // Lock robot commands
  ScopedLock lock(&robot_commands_mutex_);

  if (kDebug) {
    printf("Received commands for robots: ");
    for (int i = 0; i < message.command_size(); ++i) {
      printf("%X ", message.command(i).robot_id());
    }
    printf("\n");
  }

  const unsigned int prev_num_commands_next = num_commands_next_;
  if (message.command_size() + prev_num_commands_next > kMaxRobotsPerPacket) {
    fprintf(stderr, "ERRROR: Received commands for more than %d robots!\n",
            kMaxRobotsPerPacket);
    num_commands_next_ = kMaxRobotsPerPacket;
  } else {
    num_commands_next_ += message.command_size();
  }
  for (int i = 0; i < message.command_size() &&
                  i + prev_num_commands_next < kMaxRobotsPerPacket;
       ++i) {
    // Fill in command for robot i.
    Set(message.command(i),
        &(robot_commands_next_[i + prev_num_commands_next]));
  }
  // Set all remaining commands to idle.
  for (int i = num_commands_next_; i < kMaxRobotsPerPacket; ++i) {
    robot_commands_next_[i] = RobotCommandPacket::IdleCommand();
  }

  return true;
}

int8_t HardcodedFlatKickPower(const RadioProtocolCommand& command) {
  int8_t shot_power = 20;
  LOG(WARNING) << "Flat Kick: " << command.flat_kick();
  if (command.flat_kick() > 4) {
    LOG(WARNING) << "SHOOT!";
    LOG(WARNING) << "Robot ID: " << command.robot_id();
    switch (command.robot_id()) {
      case 7: {
        shot_power = 25;
        LOG(WARNING) << "7";
        break;
      }
      case 2: {
        shot_power = 25;
        LOG(WARNING) << "2";
        break;
      }
      case 3: {
        shot_power = 25;
        LOG(WARNING) << "3";
        break;
      }
      case 4: {
        shot_power = 17;
        LOG(WARNING) << "4";
        break;
      }
      case 6: {
        shot_power = 12;
        LOG(WARNING) << "6";
        break;
      }
      case 1: {
        shot_power = 10;
        LOG(WARNING) << "6";
        break;
      }
      default: {
        shot_power = 15;
        LOG(WARNING) << "default";
        break;
      }
    }
  } else {
    switch (command.robot_id()) {
      case 7: {
        shot_power = 12;
        LOG(WARNING) << "7";
        break;
      }
      case 2: {
        shot_power = 14;
        LOG(WARNING) << "2";
        break;
      }
      case 3: {
        shot_power = 14;
        LOG(WARNING) << "3";
        break;
      }
      case 1: {
        shot_power = 6;
        LOG(WARNING) << "6";
        break;
      }

      default: {
        shot_power = 7;
        break;
      }
    }
  }
  return shot_power;
}

int8_t LearnedFlatKickPower(
    const RadioProtocolCommand& command,
    const std::array<KickCurve, kNumSSLVisionIds>& kick_curves_) {
  NP_CHECK(command.robot_id() < static_cast<int>(kick_curves_.size()) &&
           command.robot_id() >= 0);
  // Converts from m/s given command to mm/s command that we used to compute the
  // curve.
  return kick_curves_[command.robot_id()].GetKickPower(command.flat_kick() *
                                                       1000.0f);
}

int8_t GetKickPower(
    const RadioProtocolCommand& command,
    const std::array<KickCurve, kNumSSLVisionIds>& kick_curves_) {
  if (kKickTune) {
    return static_cast<int8_t>(math_util::Clamp(
        command.flat_kick(), 0.0f,
        static_cast<float>(std::numeric_limits<int8_t>::max())));
  }
  if (!kRadioUsePassedPower) {
    return HardcodedFlatKickPower(command);
  }
  return LearnedFlatKickPower(command, kick_curves_);
}

void MinutebotRadio::Set(const RadioProtocolCommand& command,
                         RobotCommandPacket* cmd) {
  static const bool kDebug = false;
  // Set robot id in flags
  cmd->flags = command.robot_id();

  //   cmd->flags = (uint8_t)command.robot_id();
  //   cmd->flags = 2;
  // radio_command->id = idToYisibotIndex(command.robot_id());

  // Calculate velocity from input velocity (why multiply by these numbers)
  float command_x_velocity = command.velocity_x();
  // Positive Y is right
  float command_y_velocity = command.velocity_y();
  float command_rotational_velocity = command.velocity_r();

  if (std::isnan(command_x_velocity) || std::isinf(command_x_velocity)) {
    command_x_velocity = 0;
  }
  if (std::isnan(command_y_velocity) || std::isinf(command_y_velocity)) {
    command_y_velocity = 0;
  }
  if (std::isnan(command_rotational_velocity) ||
      std::isinf(command_rotational_velocity)) {
    command_rotational_velocity = 0;
  }

  //   std::printf("Command_x_velocity %f\n", command_x_velocity);
  //   std::printf("Command_y_velocity %f\n", command_y_velocity);
  //   std::printf("Command_r_velocity %f\n", command_rotational_velocity);

  constexpr float wheel_angles[] = {
      DegToRad<float>(90 + 54), DegToRad<float>(90 + 135),
      DegToRad<float>(90 - 135), DegToRad<float>(90 - 54)};

  if (kDebug) {
    printf("Robot: %d Command x: %f, y: %f, theta: %f", command.robot_id(),
           command_x_velocity, command_y_velocity, command_rotational_velocity);
    printf("Speed: %f",
           std::sqrt(Sq(command_x_velocity) + Sq(command_y_velocity)));
  }

  if (!kUseCmdCorrection) {
    static constexpr float kCommandScalingConstant = 1.00;
    // Converting to a value range used for command output.
    command_x_velocity *= (kCommandScalingConstant * (127.0f / 4.0f));
    command_y_velocity *= (kCommandScalingConstant * (127.0f / 4.0f));
    command_rotational_velocity *= (127.0f / 50.63f);

    // printf("%f %f %f\n", x_velocity, y_velocity, r_velocity);
    for (int i = 0; i < 4; ++i) {
      // static_cast<int>() truncates the decimal place, meaning
      // that the magnitude is rounded the same for both positive and negative
      // values, which a floor() call would not provide.
      const int v =
          -static_cast<int>(command_x_velocity * cos(wheel_angles[i]) +
                            command_y_velocity * sin(wheel_angles[i]) +
                            command_rotational_velocity);
      //       if (kDebug) std::printf("%4d ", v);
      cmd->wheel_velocity[i] = v;
    }
  } else {
    Eigen::Vector4d wheel_vel(0.0, 0.0, 0.0, 0.0);
    pose_2d::Pose2Df velocity;
    velocity.translation.x() = command_x_velocity;
    velocity.translation.y() = command_y_velocity;
    velocity.angle = command_rotational_velocity;
    kinematic_model_.InverseModel(velocity, &wheel_vel, command.robot_id());

    for (int i = 0; i < 4; ++i) {
      cmd->wheel_velocity[i] = static_cast<int>(wheel_vel[i]);
    }
  }

  if (kDebug) {
    std::printf("\n");
  }

  cmd->dribble_power = 0;
  const bool dribble = command.has_dribbler_spin();
  if (dribble) {
    cmd->dribble_power = 40;
  }

  bool shoot = command.has_flat_kick();
  bool chip = command.has_chip_kick();
  if (shoot) {
    const int8_t shot_power = GetKickPower(command, kick_curves_);
    cmd->Kick(shot_power, false);
    printf("Shot power: %d\n", cmd->kick_power);
  }
  if (chip) {
    int shot_power = 90;
    if (command.chip_kick() > 4) {
      shot_power = 90;
    }
    cmd->Kick(shot_power, true);
  }
}

void PrintRadioCommands(const RadioCommandPacket& packet) {
  printf("Radio packets:\n");
  for (int i = 0; i < kMaxRobotsPerPacket; ++i) {
    printf(
        "%X: %d %d %d %d\n", packet.robots[i].flags,
        packet.robots[i].wheel_velocity[0], packet.robots[i].wheel_velocity[1],
        packet.robots[i].wheel_velocity[2], packet.robots[i].wheel_velocity[3]);
  }
}

void* MinutebotRadio::TransmitThread(void* radio_ptr) {
  static const bool kDebug = false;
  MinutebotRadio& radio = *(reinterpret_cast<MinutebotRadio*>(radio_ptr));
  RadioCommandPacket radio_packet;

  RateLoop time_helper(60);

  radio_packet.prelude[0] = 'R';
  radio_packet.prelude[1] = 'P';
  for (unsigned int i = 0; i < kMaxRobotsPerPacket; ++i) {
    radio_packet.robots[i] = RobotCommandPacket::IdleCommand();
  }

  while (!radio.shutdown_) {
    {
      ScopedLock lock(&(radio.robot_commands_mutex_));
      // Copy all available commands.
      CHECK_LE(radio.num_commands_next_, kMaxRobotsPerPacket);
      for (unsigned int i = 0; i < radio.num_commands_next_; ++i) {
        radio_packet.robots[i] = radio.robot_commands_next_[i];
      }

      // Fill the rest of the packet with null commands
      for (int i = radio.num_commands_next_; i < kMaxRobotsPerPacket; ++i) {
        radio_packet.robots[i] = RobotCommandPacket::IdleCommand();
      }

      radio_packet.ComputeCRC();

      // Print the packet
      if (kDebug) PrintRadioCommands(radio_packet);
      // Send the packet
      radio.serial_.write(&radio_packet, sizeof(radio_packet));

      radio.num_commands_next_ = 0;
    }
    time_helper.Sleep();
  }
  return NULL;
}

}  // namespace radio
