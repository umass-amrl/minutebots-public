// Copyright 2016 - 2018 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// UDP Networking Library
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

#include <string>

#ifndef SRC_NET_NETRAW_H_
#define SRC_NET_NETRAW_H_

struct sockaddr_in;

namespace google {
namespace protobuf {
class MessageLite;
}  // namespace protobuf
}  // namespace google

namespace net {

class UDPMulticastServer {
 public:
  // Default constructor, initialize variables and message buffer.
  UDPMulticastServer();

  // Default destructor. Closes interface if open.
  ~UDPMulticastServer();

  // Open a UDP multicast server interface. Returns true iff succesful.
  bool Open(const std::string& address, int port, bool receive_mode_);

  // Close the UDP multicast server interface if it is running.
  void Close();

  // Blocking function to receive raw data, return true if data was succesfully
  // received.
  bool Receive(std::string* data);

  // Convenience wrapper function to receive Google protobuf messages.
  bool ReceiveProtobuf(google::protobuf::MessageLite* message);

  // Non-blocking function to receive raw data, return true if data was
  // succesfully received.
  bool TryReceive(std::string* data);

  // Non-blocking convenience wrapper function to receive Google protobuf
  // messages.
  bool TryReceiveProtobuf(google::protobuf::MessageLite* message);

  // Broadcast send raw data, return true on success.
  bool Send(const std::string& data);

  // Broadcast send Google protobuf data, return true on success.
  bool SendProtobuf(const google::protobuf::MessageLite& message);

  // Check for state of server; return true iff the multicast server is up
  // and running.
  bool IsOpen() const;

  // Sets this server to split large messages
  void SetSplitLarge(bool split);

  // Sets a receive timeout, in microseconds. Call with timeout_us=0 to
  // disable blocking reads. Returns true if successful.
  bool SetReceiveTimeout(uint32_t timeout_us);

 private:
  // Internal Receive function.
  bool RawReceive(int flags, int* received_bytes);

  // File descriptor for socket.
  int socket_fd_;
  // UDP Multicast address.
  std::string address_;
  // UDP Port number.
  int port_number_;
  // Indicates whether the specified address is a multicast address or not.
  bool is_multicast_;
  // Pointer to multicast socket address definition.
  sockaddr_in* socket_address_ptr_;
  // Constant size of receive buffer.
  const int kReceiveBufferSize;  // 65535 // 9000 //1600
  // Max size of a message to be sent over the network;
  const int kMaxMessageSize;
  // Raw buffer to receive data.
  char* receive_buffer_;
  // Indicates that this instance is running in transmit mode.
  bool receive_mode_;
  // Indicates that this should split messages larger than kMaxMessageSize.
  bool split_large_messages_;
  // Current message id, increments as messages are sent out.
  uint32_t message_id_;
  // Timeout for socket reads, in microseconds. Set to 0 to disable.
  uint32_t receive_timeout_;
};

}  // namespace net

#endif  // SRC_NET_NETRAW_H_
