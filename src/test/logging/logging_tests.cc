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

#include <cmath>
#include <iostream>
#include "glog/logging.h"
#include "logging/logger.h"
#include "eigen3/Eigen/Core"
#include "shared/common_includes.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::PrintLog;
using MinuteBotsProto::PrintLogs;
using MinuteBotsProto::LogWrapper;
using MinuteBotsProto::TextTree;

// Test creating and adding logs to the debugger
TEST(LoggingTest, Creation) {
  logger::Logger test_log;
  SoccerDebugMessage message = test_log.ReturnMessage();
  ASSERT_EQ(message.print_logs_size(), 0);
  test_log.LogPrint(" ", " ");
  message = test_log.ReturnMessage();
  ASSERT_EQ(message.print_logs_size(), 1);
  test_log.LogPrint(" ", " ");
  message = test_log.ReturnMessage();
  ASSERT_EQ(message.print_logs_size(), 2);
}

TEST(LoggingTest, LogInt) {
  logger::Logger test_log;
  const int five = 5;
  test_log.LogPrint("Integer: %d", five);
  SoccerDebugMessage message = test_log.ReturnMessage();
//   PrintLogs logs = message.print_logs();
//   PrintLog log = logs.logs(0);
  string test = "";
  logger::BuildString(message.print_logs(0), &test);
  ASSERT_EQ(strcmp(test.c_str(), "Integer: 5"), 0);
}

TEST(LoggingTest, LogDouble) {
  logger::Logger test_log;
  const double five = 5.0;
  test_log.LogPrint("Double: %f", five);
  SoccerDebugMessage message = test_log.ReturnMessage();
//   PrintLogs logs = message.print_logs();
//   PrintLog log = logs.logs(0);
  string test = "";
  logger::BuildString(message.print_logs(0), &test);
  ASSERT_EQ(strcmp(test.c_str(), "Double: 5.000000"), 0);
}

TEST(LoggingTest, LogFloat) {
  logger::Logger test_log;
  const float five = 5.0;
  test_log.LogPrint("Float: %f", five);
  SoccerDebugMessage message = test_log.ReturnMessage();
//   PrintLogs logs = message.print_logs();
//   PrintLog log = logs.logs(0);
  string test = "";
  logger::BuildString(message.print_logs(0), &test);
  ASSERT_EQ(strcmp(test.c_str(), "Float: 5.000000"), 0);
}

TEST(LoggingTest, LogString) {
  logger::Logger test_log;
  string five = "five";
  test_log.LogPrint("String: %s", five);
  SoccerDebugMessage message = test_log.ReturnMessage();
//   PrintLogs logs = message.print_logs();
//   PrintLog log = logs.logs(0);
  string test = "";
  logger::BuildString(message.print_logs(0), &test);
  ASSERT_EQ(strcmp(test.c_str(), "String: five"), 0);
}

TEST(LoggingTest, LogPercent) {
  logger::Logger test_log;
  test_log.LogPrint("String: %%");
  SoccerDebugMessage message = test_log.ReturnMessage();
//   PrintLogs logs = message.print_logs();
//   PrintLog log = logs.logs(0);
  string test = "";
  logger::BuildString(message.print_logs(0), &test);
  ASSERT_EQ(strcmp(test.c_str(), "String: %"), 0);
}

TEST(LoggingTest, LogFixedFloat) {
  logger::Logger test_log;
  const float five = 5.0;
  test_log.LogPrint("Float: %.3f", five);
  SoccerDebugMessage message = test_log.ReturnMessage();
//   PrintLogs logs = message.print_logs();
//   PrintLog log = logs.logs(0);
  string test = "";
  logger::BuildString(message.print_logs(0), &test);
  ASSERT_EQ(strcmp(test.c_str(), "Float: 5.000"), 0);
}

