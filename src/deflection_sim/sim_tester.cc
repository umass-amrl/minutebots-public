// Copyright 2016 - 2017 kvedder@umass.edu
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
#include <glog/logging.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "constants/constants.h"
#include "eigen3/Eigen/Core"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "net/netraw.h"
#include "netraw_test_message.pb.h"
#include "radio_protocol_wrapper.pb.h"
#include "sim/networkhandler.h"
#include "sim/objects/ball.h"
#include "sim/objects/robot.h"
#include "sim/worldstate.h"
#include "util/colorize.h"
#include "util/timer.h"

using colorize::ColorGreen;
using colorize::ColorBlue;
using colorize::ColorCyan;
using Eigen::Vector2f;
using google::protobuf::RepeatedPtrField;
using net::UDPMulticastServer;
using pose_2d::Pose2Df;
using simulator::Ball;
using simulator::NetworkHandler;
using simulator::Robot;
using simulator::WorldState;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_GeometryData;
using SSLVisionProto::SSL_GeometryFieldSize;
using SSLVisionProto::SSL_WrapperPacket;
using std::cerr;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::thread;
using std::vector;

static const int kNthPrint = 5;
static const int kNumPacketsToSend = 1000;

static const float kTransmitRate = 60.0;
static const float kTimeSlice = 1.0 / kTransmitRate;

static const double kReceiveRate = 120.0;  // Hz.

static int kNumRobots = 1;

enum TestCase { CIRCLE_SPIN, LINE_DRIVE, FLIP_FLOP, KICK_BALL, STOP };

TestCase test_case = CIRCLE_SPIN;

void SetupVelocityCommands(RadioProtocolWrapper* message, int i) {
  message->clear_command();
  for (int robot_index = 0; robot_index < kNumRobots; ++robot_index) {
    RadioProtocolCommand* data = message->add_command();

    switch (test_case) {
      case KICK_BALL: {
        data->set_robot_id(robot_index);
        data->set_velocity_x(2);  // Drive forward at 2 m/s.
        data->set_velocity_y(0);
        data->set_velocity_r(0);  // Turn at 0 rad/s.
        data->set_flat_kick(3);   // Flat kick at 3 m/s.
        data->set_chip_kick(0);
        data->set_dribbler_spin(-1);  // Add max backspin.
      } break;
      case LINE_DRIVE: {
        data->set_robot_id(robot_index);
        data->set_velocity_x(3);  // Drive forward at 3 m/s.
        data->set_velocity_y(0);
        data->set_velocity_r(0);  // Turn at 0 rad/s.
        data->set_flat_kick(0);
        data->set_chip_kick(0);
        data->set_dribbler_spin(-1);  // Add max backspin.
      } break;
      case STOP: {
        data->set_robot_id(robot_index);
        // Drive forward/backward at 5 m/s.
        float velocity = 0;
        if (i < 60) {
          // Drive period.
          velocity = 5;
        } else {
          // Stop period.
          velocity = 0;
        }
        data->set_velocity_x(velocity);
        data->set_velocity_y(0);
        data->set_velocity_r(0);  // Turn at 0 rad/s.
        data->set_flat_kick(0);
        data->set_chip_kick(0);
        data->set_dribbler_spin(-1);  // Add max backspin.
      } break;
      case FLIP_FLOP: {
        data->set_robot_id(robot_index);
        // Drive forward/backward at 5 m/s.
        float velocity = 0;
        if (i % 300 < 150) {
          // First period.
          velocity = (i % 150 > 40) ? -5 : 5;
        } else {
          // Second period.
          velocity = (i % 150 > 110) ? -5 : 5;
        }
        data->set_velocity_x(velocity);
        data->set_velocity_y(0);
        data->set_velocity_r(0);  // Turn at 0 rad/s.
        data->set_flat_kick(0);
        data->set_chip_kick(0);
        data->set_dribbler_spin(-1);  // Add max backspin.
      } break;
      case CIRCLE_SPIN:  // Fallthrough intentional.
      default: {
        data->set_robot_id(robot_index);
        data->set_velocity_x(10);  // Drive forward at 10 m/s.
        data->set_velocity_y(0);
        data->set_velocity_r(10);  // Turn at 10 rad/s.
        data->set_flat_kick(0);
        data->set_chip_kick(0);
        data->set_dribbler_spin(-1);  // Add max backspin.
      } break;
    }
  }
}

void SendInputPackets() {
  UDPMulticastServer udp_server_;
  if (!udp_server_.Open(DATA_STREAM_CMD_IP, DATA_STREAM_CMD_PORT, false)) {
    LOG(ERROR) << "Error opening UDP port, exiting." << endl;
  } else {
    LOG(INFO) << "Opening UDP port successful!" << endl;
  }
  CHECK(udp_server_.IsOpen());

  LOG(INFO) << ColorGreen("Send Input Thread Setup!") << endl;

  RateLoop loop(kTransmitRate);
  RadioProtocolWrapper message;

  LOG(INFO) << ColorCyan("Number of commands in setup RadioProtocolWrapper: " +
                         std::to_string(message.command_size()))
            << endl;

  for (int i = 0; i < kNumPacketsToSend; ++i) {
    SetupVelocityCommands(&message, i);
    if (!udp_server_.SendProtobuf(message)) {
      LOG(ERROR) << "Send error" << endl;
    } else if (!(i % kNthPrint)) {
      LOG(INFO) << "Sent " << kNthPrint << " packets..." << endl;
    }
    loop.Sleep();
  }
  udp_server_.Close();
  LOG(INFO) << ColorGreen("Send Input Packets Loop Done!") << endl;
}

bool CreateSaveDirectory(const string& base_file_path, string* save_directory) {
  std::ostringstream oss;
  oss << GetWallTime();
  string time_string = oss.str();
  const int dir_err = mkdir((base_file_path + time_string).c_str(),
                            S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (dir_err == -1) {
    LOG(INFO) << "Error creating directory!" << endl;
    return false;
  }
  *save_directory = base_file_path + time_string + "/state";
  return true;
}

void RecieveOutputPackets() {
  UDPMulticastServer udp_server_;
  if (!udp_server_.Open(DATA_STREAM_VISION_IP, DATA_STREAM_VISION_PORT, true)) {
    LOG(ERROR) << "Error opening UDP port, exiting." << endl;
  } else {
    LOG(INFO) << "Opening UDP port successful!" << endl;
  }
  CHECK(udp_server_.IsOpen());

  LOG(INFO) << ColorGreen("Recieve Output Thread Setup!") << endl;

  SSL_WrapperPacket message;
  string base_path = "/tmp/SSL_simulator_run_";
  string save_prefix;
  //   if (!CreateSaveDirectory(base_path, &save_prefix)) {
  //     LOG(FATAL) << "Failed to create save directory under the base path "
  //         << base_path << endl;
  //   }
  for (int i = 0;
       udp_server_.ReceiveProtobuf(&message) && i < kNumPacketsToSend; ++i) {
    if (!(i % kNthPrint)) {
      LOG(INFO) << ColorBlue(std::to_string(kNthPrint) +
                             "th Packet recieved from simulator:")
                << endl;

      if (message.has_detection()) {
        LOG(INFO) << "SSL_Wrapper has detection data!" << endl;
        const SSL_DetectionFrame& detection = message.detection();
        const RepeatedPtrField<SSL_DetectionRobot>& robots_yellow =
            detection.robots_yellow();
        const RepeatedPtrField<SSL_DetectionRobot>& robots_blue =
            detection.robots_blue();
        const RepeatedPtrField<SSL_DetectionBall>& balls = detection.balls();
        LOG(INFO) << " - Number of Yellow Bots: " << robots_yellow.size()
                  << endl;
        LOG(INFO) << " - Number of Blue Bots: " << robots_blue.size() << endl;
        LOG(INFO) << " - Number of Balls: " << balls.size() << endl;
      } else {
        LOG(ERROR) << "SSL_Wrapper contains no detection data!" << endl;
      }

      if (message.has_geometry()) {
        LOG(INFO) << "SSL_Wrapper has geometry data!" << endl;
        const SSL_GeometryData& geometry = message.geometry();
        if (geometry.has_field()) {
          LOG(INFO) << "SSL_Wrapper has field data!" << endl;
          const SSL_GeometryFieldSize& field = geometry.field();
          LOG(INFO) << " - Length: " << field.field_length() << endl;
          LOG(INFO) << " - Width: " << field.field_width() << endl;
          LOG(INFO) << " - Goal Width: " << field.goal_width() << endl;
          LOG(INFO) << " - Goal Depth: " << field.goal_depth() << endl;
        } else {
          LOG(ERROR) << "SSL_Wrapper contains no field data!\n~";
        }
      } else {
        LOG(ERROR) << "SSL_Wrapper contains no geometry data!" << endl;
      }
      LOG(INFO) << ColorBlue(
                       "RecieveOutputPackets recieved packet from simulator!")
                << endl;
    }
  }
  udp_server_.Close();
  LOG(INFO) << ColorGreen("Recieve Output Packets Loop Done!") << endl;
}

void CreateRobots(vector<Robot>* robots) {
  for (int i = 0; i < kNumRobots; ++i) {
    const Pose2Df pose(0.0, Vector2f(1000 * i - 3000, 0));
    const Vector2f velocity(0, 0);
    const int id = i;
    const float max_translation = 10;  // meters/s
    const float max_accel = 0.001;     // m/s^2
    const float radius = 90;           // mm
    robots->push_back(Robot(pose, velocity, id, kTimeSlice, max_translation,
                            max_accel, radius));
  }
}

int main(int argc, char** argv) {
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;   // INFO level logging.
  FLAGS_colorlogtostderr = 1;  // Colored logging.
  FLAGS_logtostderr = true;    // Don't log to disk.

  if (argc != 3 || strlen(argv[1]) != 2 || argv[1][0] != '-' ||
      argv[2][0] != '-') {
    cout << "Usage: sim_tester -[c, l, f, k, s] -[Number of robots]\n";
    exit(0);
  }

  switch (argv[1][1]) {
    case 'k':
      test_case = KICK_BALL;
      break;
    case 'l':
      test_case = LINE_DRIVE;
      break;
    case 'f':
      test_case = FLIP_FLOP;
      break;
    case 'c':
      test_case = CIRCLE_SPIN;
      break;
    case 's':
      test_case = STOP;
      break;
    default:
      cout << "Invalid test type!!!\n";
      exit(0);
      break;
  }

  if (argv[2][1] >= '1' && argv[2][1] <= '9') {
    kNumRobots = argv[2][1] - '0';
  } else {
    cout << "Invalid number of robots!!!\n";
    exit(0);
  }

  LOG(INFO) << ColorGreen("Starting sim_tester!") << endl;

  vector<Robot> robots;
  CreateRobots(&robots);
  Ball ball(Pose2Df(0, Vector2f(0, 0)), Vector2f(0, 0), 0.08f, kTimeSlice,
            0.0005f);
  WorldState world_state(robots, ball);
  NetworkHandler reciever(&world_state, DATA_STREAM_CMD_IP,
                          DATA_STREAM_CMD_PORT, DATA_STREAM_VISION_IP,
                          DATA_STREAM_VISION_PORT, kTransmitRate, kReceiveRate);
  reciever.Start();

  thread send_input_packets_thread(SendInputPackets);
  thread recieve_output_packets_thread(RecieveOutputPackets);

  send_input_packets_thread.join();
  LOG(INFO) << ColorGreen("Send Input Packets terminated!") << endl;
  recieve_output_packets_thread.join();
  LOG(INFO) << ColorGreen("Recieve Output Packets terminated!") << endl;

  google::protobuf::ShutdownProtobufLibrary();
  LOG(INFO) << ColorGreen("Shutdown protobuf library!") << endl;
  google::FlushLogFiles(google::GLOG_INFO);
  LOG(INFO) << ColorGreen("Flushed log files!") << endl;
  LOG(INFO) << ColorGreen("Exiting...") << endl;
}
