// Copyright 2016 - 2019 joydeepb@cs.umass.edu
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


#include "src/net/netraw.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <glog/logging.h>
#include <google/protobuf/message_lite.h>
#include <utility>
#include <vector>
#include "shared/common_includes.h"
#include "util/helpers.h"

using google::protobuf::MessageLite;

namespace {
void PrintError(const string& error, const string& address, int port) {
  const string error_prompt =
      StringPrintf("Network error for %s:%d while %s",
                   address.c_str(),
                   port,
                   error.c_str());
  perror(error_prompt.c_str());
}
}  // namespace

namespace net {

bool IsMulticast(const string& address) {
  sockaddr_in ip;
  // Convert string to IP address struct.
  if (inet_pton(AF_INET, address.c_str(), &(ip.sin_addr)) != 1) {
    PrintError("checking multicast", address, 0);
    return false;
  }
  const uint32_t ip_number = static_cast<uint32_t>(ip.sin_addr.s_addr);
  const uint32_t ip_high_nibble = ((ip_number & 0xF0u) >> 4);
  const bool is_multicast = (ip_high_nibble == 0b1110u);
  return is_multicast;
}

bool IsLocalhost(const in_addr& address) {
  const uint32_t ip_number = static_cast<uint32_t>(address.s_addr);
  const uint32_t ip_high_byte = ip_number & 0xFFu;
  const bool is_localhost = (ip_high_byte == 0x7F);
  return is_localhost;
}

UDPMulticastServer::UDPMulticastServer() :
    socket_fd_(0),
    port_number_(0),
    socket_address_ptr_(nullptr),
    kReceiveBufferSize(500000),
    kMaxMessageSize(60000),
    receive_buffer_(nullptr),
    receive_mode_(true),
    split_large_messages_(false),
    message_id_(0),
    receive_timeout_(0) {
  receive_buffer_ = new char[kReceiveBufferSize];
}

UDPMulticastServer::~UDPMulticastServer() {
  Close();
  if (socket_address_ptr_) delete socket_address_ptr_;
  delete[] receive_buffer_;
}

bool UDPMulticastServer::Open(const string& address,
                              int port,
                              bool receive_mode) {
  address_ = address;
  port_number_ = port;
  receive_mode_ = receive_mode;
  is_multicast_ = IsMulticast(address);
  // Open a UDP socket.
  socket_fd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    PrintError("opening UDP socket", address_, port_number_);
    socket_fd_ = 0;
    return false;
  }
  // Allow reuse of the socket.
  const u_int yes = 1;
  if (setsockopt(socket_fd_,
                SOL_SOCKET,
                SO_REUSEADDR,
                &yes,
                sizeof(yes)) < 0) {
    PrintError("setting SO_REUSEADDR", address_, port_number_);
    Close();
    return false;
  }
  // Save the multicast address for future Send() operations.
  if (socket_address_ptr_ == nullptr) {
    socket_address_ptr_ = new struct sockaddr_in;
  }
  memset(socket_address_ptr_, 0, sizeof(*socket_address_ptr_));
  socket_address_ptr_->sin_family = AF_INET;
  socket_address_ptr_->sin_addr.s_addr = inet_addr(address_.c_str());
  socket_address_ptr_->sin_port = htons(port_number_);
  // Bind the socket to the address for subsequent receives.
  if (receive_mode_) {
    if (bind(socket_fd_,
            reinterpret_cast<struct sockaddr*>(socket_address_ptr_),
            sizeof(*socket_address_ptr_)) < 0) {
      PrintError("setting bind", address_, port_number_);
      Close();
      return false;
    }
  }
  socket_address_ptr_->sin_port = htons(port_number_);
  // Request the kernel for multicast membership.
  // if (true) {
  if (is_multicast_) {
    ip_mreq multicast_request;
    multicast_request.imr_multiaddr.s_addr = inet_addr(address_.c_str());
    multicast_request.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(socket_fd_,
                    IPPROTO_IP,
                    IP_ADD_MEMBERSHIP,
                    &multicast_request,
                    sizeof(multicast_request)) < 0) {
      PrintError("adding multicast membership", address_, port_number_);
      Close();
      return false;
    }
  }
  return true;
}

void UDPMulticastServer::SetSplitLarge(bool split) {
  split_large_messages_ = split;
}

void PrintBufferInHex(const char* buffer, int size) {
  printf("Size: %d\n", size);
  for (int i = 0; i < size; i++) {
    printf("%X", buffer[i]);
  }
  std::cout << std::endl;
//   for (int i = 0; i < size; i++) {
//     std::cout << buffer[i];
//   }
  std::cout << std::endl;
}

bool UDPMulticastServer::Send(const string& data) {
  if (receive_mode_) {
    fprintf(stderr,
            "ERROR: UDPMulticastServer in receive mode sending data!\n");
  }
  if (socket_fd_ == 0) return false;
  // Normal case in which the network handler will not split messages.
  if (!split_large_messages_) {
    const int bytes_sent =
        sendto(socket_fd_,
              data.data(),
              data.size(),
              0,
              reinterpret_cast<struct sockaddr*>(socket_address_ptr_),
              sizeof(*socket_address_ptr_));
    if (bytes_sent < 0 || bytes_sent != static_cast<int>(data.size())) {
      const string error =
          StringPrintf("sending %d bytes to %s:%d, only %d sent\n",
                      static_cast<int>(data.size()),
                      address_.c_str(),
                      port_number_,
                      bytes_sent);
      PrintError(error, address_, port_number_);
      return false;
    }
  // Behavior for splitting messages larger than a set size.
  } else {
    const int size = data.length();
    const int parts = ceil(static_cast<float>(size) /
        static_cast<float>((kMaxMessageSize - 2)));
    unsigned char parts_char = parts;
    int current = 0;
    unsigned char count = 1;
    // Split the message up and send packets until all is sent.
    while (current < size) {
      string partial = data.substr(current, kMaxMessageSize);
      const char* temp = reinterpret_cast<const char*>(&message_id_);
      uint32_t test = 5;
      memcpy(&test, temp, 4);
      for (uint i = 0; i < 4; ++i) {
        partial += temp[i];
      }
      partial += count;
      partial += parts_char;
      count += 1;
      current += kMaxMessageSize;
      sendto(socket_fd_,
             partial.data(),
             partial.size(),
             0,
             reinterpret_cast<struct sockaddr*>(socket_address_ptr_),
             sizeof(*socket_address_ptr_));
    }
    message_id_++;
  }
  return true;
}

bool UDPMulticastServer::SendProtobuf(const MessageLite& message) {
  const string data = message.SerializeAsString();
  return (Send(data));
}

void UDPMulticastServer::Close() {
  if (socket_fd_ != 0) {
    close(socket_fd_);
    socket_fd_ = 0;
  }
}

bool UDPMulticastServer::IsOpen() const {
  return (socket_fd_ > 0);
}

bool UDPMulticastServer::RawReceive(int flags, int* received_bytes) {
  static const bool kDebug = true;
  sockaddr_in source_address;
  memset(reinterpret_cast<void*>(&source_address), 0, sizeof(source_address));
  socklen_t source_address_length = sizeof(source_address);
  if (!split_large_messages_) {
    *received_bytes = recvfrom(
        socket_fd_,
        reinterpret_cast<void*>(receive_buffer_),
        kReceiveBufferSize,
        flags,
        reinterpret_cast<sockaddr*>(&source_address),
        &source_address_length);
    if (kDebug) {
      if (*received_bytes > 0) {
        char str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(source_address.sin_addr), str, INET_ADDRSTRLEN);
      }
    }
    if (*received_bytes < 0 && flags == 0) {
      return false;
    }
  } else {
    int received = 1;
    int last_part_id = -1;
    int num_parts = 999;
    uint32_t last_message_id = 0;
    std::vector<char> full_message;
    while (received > 0 && last_part_id < num_parts) {
      received = recvfrom(
          socket_fd_,
          reinterpret_cast<void*>(receive_buffer_),
          kReceiveBufferSize,
          flags,
          reinterpret_cast<sockaddr*>(&source_address),
          &source_address_length);
      if (received > 0) {
        int part_id = receive_buffer_[received - 2];
        int total = receive_buffer_[received - 1];
        uint32_t message_id = 0;
        memcpy(&message_id,
               receive_buffer_ + (received - 6),
               sizeof(message_id));
        num_parts = total;
        if (message_id != last_message_id || !(part_id > last_part_id)) {
          if (part_id == 1) {
            full_message.clear();
          } else {
            return false;
          }
        }
        char* end = receive_buffer_ + (received - 6);
        full_message.insert(full_message.end(), receive_buffer_, end);
        last_message_id = message_id;
        last_part_id = part_id;
      }
    }
    if (last_part_id == num_parts) {
      *received_bytes = static_cast<int>(full_message.size());
      std::memcpy(receive_buffer_,
                  reinterpret_cast<char*>(full_message.data()),
                  *received_bytes);
    } else {
      return false;
    }
  }
  return true;
}

bool UDPMulticastServer::Receive(string* data) {
  DCHECK(data != nullptr);
  int received_bytes = 0;
  if (!RawReceive(0, &received_bytes)) {
    return false;
  }
  data->assign(receive_buffer_, received_bytes);
  return true;
}

bool UDPMulticastServer::ReceiveProtobuf(MessageLite* message) {
  DCHECK(message != nullptr);
  int received_bytes = 0;
  if (!RawReceive(0, &received_bytes)) {
    return false;
  }
  message->ParseFromArray(receive_buffer_, received_bytes);
  return true;
}

bool UDPMulticastServer::TryReceive(string* data) {
  DCHECK(data != nullptr);
  int received_bytes = 0;
  // If there is a specified timeout, don't use MSG_DONTWAIT.
  const int receive_flags = (receive_timeout_ > 0) ? 0 : MSG_DONTWAIT;
  if (!RawReceive(receive_flags, &received_bytes)) {
    return false;
  } else if (received_bytes == 0) {
    // Not an error, there just isn't any data available to read right now.
    return false;
  }
  data->assign(receive_buffer_, received_bytes);
  return true;
}

bool UDPMulticastServer::TryReceiveProtobuf(MessageLite* message) {
  DCHECK(message != nullptr);
  int received_bytes = 0;
  errno = 0;
  // If there is a specified timeout, don't use MSG_DONTWAIT.
  const int receive_flags = (receive_timeout_ > 0) ? 0 : MSG_DONTWAIT;
  int return_val = RawReceive(receive_flags, &received_bytes);
  int errsv = errno;
  if (split_large_messages_) {
    if (!return_val) {
      return false;
    } else if (received_bytes == 0) {
      // Not an error, there just isn't any data available to read right now.
      return false;
    } else if (received_bytes < 0) {
      // Reading failed, logging with errno.
      LOG(WARNING) << "Reading in TryRecieveProtobuf() failed with errno: "
          << errsv << " (" << strerror(errsv) << ")!\n";
      return false;
    }
  } else {
    if (!return_val) {
      return false;
    } else if (received_bytes == 0 || errsv == EAGAIN) {
      // Not an error, there just isn't any data available to read right now.
      return false;
    } else if (received_bytes < 0) {
      // Reading failed, logging with errno.
      LOG(WARNING) << "Reading in TryRecieveProtobuf() failed with errno: "
      << errsv << " (" << strerror(errsv) << ")!\n";
      return false;
    }
  }
  try {
    message->ParseFromArray(receive_buffer_, received_bytes);
  } catch(...) {
    LOG(WARNING) << "Bad protobuff received.";
    return false;
  }
  return true;
}

bool UDPMulticastServer::SetReceiveTimeout(uint32_t timeout_us) {
  receive_timeout_ = timeout_us;
  struct timeval timeout;
  timeout.tv_sec = receive_timeout_ / 1000000;
  timeout.tv_usec = receive_timeout_ % 1000000;

  const int err_code = setsockopt(
      socket_fd_,
      SOL_SOCKET,
      SO_RCVTIMEO,
      reinterpret_cast<void*>(&timeout),
      sizeof(timeout));
  if (err_code < 0) {
    PrintError("Error setting receive timeout", address_, port_number_);
    return false;
  }
  return true;
}


}  // namespace net
