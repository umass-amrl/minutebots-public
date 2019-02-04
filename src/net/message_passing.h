// Copyright 2016 - 2017 joydeepb@cs.umass.edu
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
// Message-passing library.
//

#ifndef SRC_NET_MESSAGE_PASSING_H_
#define SRC_NET_MESSAGE_PASSING_H_

namespace message_passing {

enum struct MessageTopics : int {
  kVisionTopic = 1,
  kRobotControlTopic = 2,
  kRobotFeedbackTopic = 3,
  kRefereeTopic = 4,
  kDebuggingTopic = 5
};

class MessagePassing {
 public:
  // Default constructor.
  MessagePassing();

  // Register to a receive from a topic. Returns true on success.
  bool RegisterSubscriber();

  // Register to a send to a topic. Returns true on success.
  bool RegisterPublisher();

  // Publish data to a topic. Returns true on success.
  bool Publish();
};

}  // namespace message_passing

#endif  // SRC_NET_MESSAGE_PASSING_H_
