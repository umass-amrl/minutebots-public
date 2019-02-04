// Copyright 2017 - 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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

#ifndef SRC_DATASTRUCTURES_BOUNDED_QUEUE_H_
#define SRC_DATASTRUCTURES_BOUNDED_QUEUE_H_

#include <vector>

#include "constants/constants.h"
#include "util/timer.h"

namespace datastructures {
template <typename Value>
class BoundedQueue {
 private:
  size_t max_size_;
  size_t current_size_;
  size_t tail_index_;
  std::vector<Value> manual_queue_;

 public:
  BoundedQueue() = delete;
  explicit BoundedQueue(unsigned int size)
      : max_size_(size),
        current_size_(0),
        tail_index_(0),
        manual_queue_(size) {}

  BoundedQueue(const BoundedQueue<Value>& other) = default;
  BoundedQueue(BoundedQueue<Value>&& other) = default;

  ~BoundedQueue() {}

  // 0th element is the oldest element in the queue.
  Value& operator[](size_t i) {
    return manual_queue_[WrapIndex(tail_index_ +  i)];
  }

  Value& UnderlyingRead(int i) {
    return manual_queue_[i];
  }

  BoundedQueue<Value>& operator=(const BoundedQueue<Value>& other) = default;
  BoundedQueue<Value>& operator=(BoundedQueue<Value>&& other) = default;

  inline size_t WrapIndex(const size_t unwrapped_index) const {
    return unwrapped_index % max_size_;
  }

  void Add(Value v) {
    manual_queue_[WrapIndex(tail_index_ + current_size_)] = v;
    if (current_size_ >= max_size_) {
      tail_index_ = WrapIndex(tail_index_ + 1);
    } else {
      current_size_++;
    }
  }

  const Value& Last() const { return manual_queue_[tail_index_]; }

  const Value& First() const {
    return manual_queue_[WrapIndex(tail_index_ + current_size_ - 1)];
  }

  const size_t Size() const { return current_size_; }

  void Clear() {
    manual_queue_.clear();
    current_size_ = 0,
    tail_index_ = 0;
  }
};

}  // namespace datastructures
#endif  // SRC_DATASTRUCTURES_BOUNDED_QUEUE_H_
