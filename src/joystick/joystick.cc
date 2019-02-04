// Copyright 2017-2018 slane@cs.umass.edu
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


#include "joystick/joystick.h"

#include <linux/joystick.h>
#include <sys/poll.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include "state/shared_state.h"
#include "constants/constants.h"

STANDARD_USINGS;
using state::SharedState;
using state::SharedRobotState;
using MinuteBotsProto::SSL_Referee;

namespace joystick {
static const bool Debug = false;

Joystick::Joystick() : MaxAxisVal(32767), MaxAxisValInv(1.0 / MaxAxisVal) {
  fd = -1;
  model = NULL;
  model_size = 0;

  num_axes = num_buttons = 0;
  for (size_t i = 0; i < MAX_NAME_LEN; ++i) {
    name[i] = 0;
  }
  for (size_t i = 0; i < MAX_AXES; ++i) {
    axes[i] = 0;
    vaxes[i] = 0;
  }
  for (size_t i = 0; i < MAX_BUTTONS; ++i) {
    buttons[i] = 0;
  }
}

bool Joystick::IsJoy(const char *key) {
  bool match = (strstr(name, key) != NULL);
  if (Debug) printf("name=[%s] key=[%s] ret=%d\n", name, key, match);
  return(match);
}

bool Joystick::Open(const char *dev) {
  static const bool debug = false;
  if (debug) printf("opening %s\n", dev);

  Close();
  fd = ::open(dev, O_RDONLY);
  if (fd < 0) return(false);

  ioctl(fd, JSIOCGAXES,               &num_axes);
  ioctl(fd, JSIOCGBUTTONS,            &num_buttons);
  ioctl(fd, JSIOCGNAME(MAX_NAME_LEN), name);

  if (num_axes > MAX_AXES) num_axes = MAX_AXES;
  if (num_buttons > MAX_BUTTONS) num_buttons = MAX_BUTTONS;

  return(true);
}

bool Joystick::Open(int joy_idx) {
  static const int DevMax = 32;
  char dev[DevMax];

  snprintf(dev, DevMax, "/dev/input/js%d", joy_idx);
  if (Open(dev)) return(true);

  snprintf(dev, DevMax, "/dev/js%d", joy_idx);
  if (Open(dev)) return(true);

  return(false);
}

int Joystick::ProcessEvents(int max_wait_time_ms) {
  struct pollfd polls[1];
  int total_num_events = 0;
  int num_events;
  struct js_event jse;

  if (fd == -1) return(-1);

  polls[0].fd = fd;
  polls[0].events = POLLIN | POLLPRI;
  polls[0].revents = 0;

  while ((num_events = poll(polls, 1, max_wait_time_ms)) > 0) {
    total_num_events += num_events;
    if (polls[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
      fprintf(stderr, "error returned from poll on joystick\n");
      return(-1);
    }

    if (polls[0].revents & (POLLIN | POLLPRI)) {
      if (read(fd, &jse, sizeof(jse)) != sizeof(jse)) {
        perror("read failed on pollable joystick\n");
        return(-1);
      }

      switch (jse.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
          if (Debug) printf("axis %d now has value %d\n", jse.number,
                            jse.value);
          axes[jse.number] = jse.value;
          break;
        case JS_EVENT_BUTTON:
          if (Debug) printf("button %d now has value %d\n", jse.number,
                           jse.value);
          buttons[jse.number] = jse.value;
          break;
      }
    }
  }

  if (num_events < 0) {
    perror("Unable to poll joystick");
    return(-1);
  }

  ApplyModel();

  return(total_num_events);
}


float ApplyDeadZone(float axis, float dead_zone, float scale) {
  if (axis < dead_zone && axis >  -dead_zone) {
    return 0;
  }
  if (axis > dead_zone) {
    return (scale * (axis - dead_zone));
  }
  if (axis < -dead_zone) {
    return (scale * (axis + dead_zone));
  }
  return 0;
}

void Joystick::MakeCommand(int robot_id, RadioProtocolCommand* command) {
  command->set_robot_id(robot_id);
  command->set_velocity_x(0);
  command->set_velocity_y(0);
  command->set_velocity_r(0);
  // for (int i = 0; i < 360; i++) {
  if (ProcessEvents(0) < 0) {
    return;
  }

  // Left bumper for chip kick
  // Right bumper for shoot
  // Right trigger for dribble forwards
  // Left trigger for dribble backwards
  if (buttons[4]) {
    command->set_chip_kick(0.5);
  } else if (buttons[5]) {
    command->set_flat_kick(6.0);
  }

  if (axes[5] == 32767) {
    command->set_dribbler_spin(1);
  } else if (axes[2] == 32767) {
    command->set_dribbler_spin(-1);
  }

  int kDeadzone = 6000;



  const float x_velocity = ApplyDeadZone(
      -static_cast<float>(axes[4]),
      kDeadzone,
      4.0f / (32768.0f - kDeadzone));

  const float y_velocity = ApplyDeadZone(
      -static_cast<float>(axes[3]),
      kDeadzone,
      4.0f / (32768.0f - kDeadzone));

  const float r_velocity = ApplyDeadZone(
      -static_cast<float>(axes[0]),
      kDeadzone,
      6.0f / (32768.0f - kDeadzone));

  command->set_velocity_x(x_velocity);
  command->set_velocity_y(y_velocity);
  command->set_velocity_r(r_velocity);
}

bool Joystick::MakeRefCommand(
      SSL_Referee::Command* command) {
  bool updated = false;
  if (ProcessEvents(0) < 0) {
    return false;
  }
  // A
  if (buttons[0]) {
    *command = MinuteBotsProto::SSL_Referee_Command_FORCE_START;
    updated = true;
  // B
  } else if (buttons[1]) {
    *command = MinuteBotsProto::SSL_Referee_Command_HALT;
    updated = true;
  // X
  } else if (buttons[2]) {
    *command = MinuteBotsProto::SSL_Referee_Command_NORMAL_START;
    updated = true;
  // Y
  } else if (buttons[3]) {
    *command = MinuteBotsProto::SSL_Referee_Command_STOP;
    updated = true;
  // LB
  } else if (buttons[4]) {
    *command = MinuteBotsProto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW;
    updated = true;
  // RB
  } else if (buttons[5]) {
    *command = MinuteBotsProto::SSL_Referee_Command_BALL_PLACEMENT_BLUE;
    updated = true;
  // back
  } else if (buttons[6]) {
    *command = MinuteBotsProto::SSL_Referee_Command_TIMEOUT_YELLOW;
    updated = true;
  // start
  } else if (buttons[7]) {
    *command = MinuteBotsProto::SSL_Referee_Command_TIMEOUT_BLUE;
    updated = true;
  // L3
  } else if (buttons[8]) {
    // No Action currently
  // R3
  } else if (buttons[9]) {
    // No Action Currently
  // XBOX button
  } else if (buttons[10]) {
  // Left
  } else if (buttons[11]) {
    *command = MinuteBotsProto::SSL_Referee_Command_BALL_PLACEMENT_YELLOW;
    updated = true;
  // Right
  } else if (buttons[12]) {
    *command = MinuteBotsProto::SSL_Referee_Command_BALL_PLACEMENT_BLUE;
    updated = true;
  // Up
  } else if (buttons[13]) {
    // No Action Currently
  // Down
  } else if (buttons[14]) {
    // No Action Currently
  // L-Left UP
  } else if (axes[0] < -20000 && axes[1] < -20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_PREPARE_KICKOFF_BLUE;
    updated = true;
  // L-Righ UP
  } else if (axes[0] > 20000 && axes[1] < -20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_PREPARE_KICKOFF_YELLOW;
    updated = true;
    // L-LeftDown
  } else if (axes[0] < -20000 && axes[1] > 20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_PREPARE_PENALTY_BLUE;
    updated = true;
    //  L-RightDown
  } else if (axes[0] > 20000 && axes[1] > 20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_PREPARE_PENALTY_YELLOW;
    updated = true;
    //  R - Left Down
  } else if (axes[3] < -20000 && axes[4] < -20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_INDIRECT_FREE_BLUE;
    updated = true;
    // L-Right Up
  } else if (axes[3] > 20000 && axes[4] < -20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_INDIRECT_FREE_YELLOW;
    updated = true;
    // L-LeftDown
  } else if (axes[3] < -20000 && axes[4] > 20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_DIRECT_FREE_BLUE;
    updated = true;
    //  L-RightDown
  } else if (axes[3] > 20000 && axes[4] > 20000) {
    *command = MinuteBotsProto::SSL_Referee_Command_DIRECT_FREE_YELLOW;
    updated = true;
  // Right Trigger
  } else if (axes[5] == 32767) {
    *command = MinuteBotsProto::SSL_Referee_Command_GOAL_BLUE;
    updated = true;
  } else if (axes[2] == 32767) {
    *command = MinuteBotsProto::SSL_Referee_Command_GOAL_YELLOW;
    updated = true;
  }
  return updated;
}

pose_2d::Pose2Df Joystick::GetVelocity() {
  float x_velocity = -static_cast<float>(axes[4])/32768.0f*kMaxRobotVelocity;
  // float x_velocity = -static_cast<float>(axes[4])/32768.0f*1000.0f;
  if (axes[4] < kDeadzone && axes[4] >= -kDeadzone) {
    x_velocity = 0;
  }

  float y_velocity = -static_cast<float>(axes[3])/32768.0f*kMaxRobotVelocity;
  // float y_velocity = -static_cast<float>(axes[3])/32768.0f*1000.0f;
  if (axes[3] < kDeadzone && axes[3] >= -kDeadzone) {
    y_velocity = 0;
  }

  float r_velocity = -static_cast<float>(axes[0])/32768.0f*kMaxRobotRotVel;
  if (axes[0] < kDeadzone && axes[0] >= -kDeadzone) {
    r_velocity = 0;
  }

  return pose_2d::Pose2Df(r_velocity, x_velocity, y_velocity);
}


void Joystick::SetModel(const Model *_model) {
  // look for terminator at end of model
  int n = 0;
  while (n < MaxModelSize && _model[n].type < 2) n++;

  SetModel(_model, n);
}

void Joystick::ApplyModel() {
  static const bool debug = false;
  if (!model) return;

  // clear current virtual axes
  for (int i = 0; i < MAX_VIRT_AXES; i++) vaxes[i] = 0.0;
  unsigned old_hold = hold;
  hold = 0;

  if (debug) {
    for (int i = 0; i < num_buttons; i++) {
      printf("%d", buttons[i]);
    }
    printf("\n");
  }

  // apply model
  for (int i = 0; i < model_size; i++) {
    Model m = model[i];
    float v = 0.0;

    if (m.type) {
      v = buttons[m.index];
      if (v > 0.0) hold |= m.vbutton;
    } else {
      if (abs(axes[m.index]) > m.min_value) {
        v = axes[m.index]*MaxAxisValInv;
        if (axes[m.index] >= 0) {
          v -= m.min_value * MaxAxisValInv;
        } else {
          v += m.min_value * MaxAxisValInv;
        }
      }
    }

    float a = v * m.weight + m.bias;
    // if(a > 0.0) vbutton |= m.vbutton;

    if (debug) {
      printf("%d:(%d,%d) %f*%f + %f = %f\n",
             m.vaxis, m.type, m.index,
             v, m.weight, m.bias, a);
    }

    if (m.vaxis < MAX_VIRT_AXES) vaxes[m.vaxis] += a;
  }

  press   = (hold) & (~old_hold);
  release = (~hold) & (old_hold);

  if (debug) {
    int i, n = num_axes;
    while (n > 0 && vaxes[n-1] == 0.0) n--;

    printf("vaxes:");
    for (i = 0; i < n; i++) {
      printf("  %d:%+5.2f", i, vaxes[i]);
    }
    printf("\n");
  }

  if (false) {
    int MaxBits = 16;

    printf("press[");
    for (int i = 0; i < MaxBits; i++) printf("%d", (press >> i)&0x1);
    printf("] ");

    printf("release[");
    for (int i = 0; i < MaxBits; i++) printf("%d", (release >> i)&0x1);
    printf("] ");

    printf("hold[");
    for (int i = 0; i < MaxBits; i++) printf("%d", (hold >> i)&0x1);
    printf("]\n");
  }
}

int Joystick::Close() {
  if (fd >= 0) ::close(fd);

  return(1);
}
}  // namespace joystick
