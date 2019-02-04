// Copyright 2018 kvedder@umass.edu
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

#ifndef SRC_DATASTRUCTURES_VECTOR_PRIORITY_QUEUE_H_
#define SRC_DATASTRUCTURES_VECTOR_PRIORITY_QUEUE_H_

#include <algorithm>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "constants/constants.h"

namespace datastructures {
template <class T, typename Comp = std::less<T>>
class VectorPriorityQueue {
 public:
  VectorPriorityQueue() = default;
  explicit VectorPriorityQueue(const std::vector<T>& v_arg) : v(v_arg) {
    std::make_heap(v.begin(), v.end());
  }
  VectorPriorityQueue(const std::vector<T>& v1, const std::vector<T>& v2)
      : v(v1) {
    v.insert(v.end(), v2.begin(), v2.end());
    std::make_heap(v.begin(), v.end(), Comp());
  }

  std::vector<T>* GetMutableVector() { return &v; }

  const std::vector<T>& GetVector() const { return v; }

  void RebuildHeap() { std::make_heap(v.begin(), v.end(), Comp()); }

  bool RemoveIfFound(const T& e) {
    auto find_result = std::find(v.begin(), v.end(), e);
    if (find_result == v.end()) {
      return false;
    }
    v.erase(find_result);
    RebuildHeap();
    return true;
  }

  bool In(const T& e) const {
    return std::find(v.begin(), v.end(), e) != v.end();
  }

  bool Update(const T& e) {
    auto find_result = std::find(v.begin(), v.end(), e);
    if (find_result == v.end()) {
      return false;
    }
    (*find_result) = e;
    RebuildHeap();
    return true;
  }

  void UpdateOrPush(const T& e) {
    auto find_result = std::find(v.begin(), v.end(), e);
    if (find_result == v.end()) {
      Push(e);
      return;
    }
    (*find_result) = e;
    RebuildHeap();
    return;
  }

  void Push(const T& e) {
    v.push_back(e);
    std::push_heap(v.begin(), v.end(), Comp());
  }
  void push(const T& e) { Push(e); }

  void PushAll(const std::vector<T>& e) {
    v.insert(v.end(), e.begin(), e.end());
    std::make_heap(v.begin(), v.end(), Comp());
  }

  void MergeOther(const VectorPriorityQueue& q) { PushAll(q.GetVector()); }

  bool Empty() { return v.empty(); }
  bool empty() { return Empty(); }

  size_t Size() { return v.size(); }
  size_t size() { return v.size(); }

  const T& Top() const {
    if (!kProduction) {
      if (v.empty()) {
        LOG(FATAL) << "Cannot get the top element of an empty queue";
      }
    }
    return v[0];
  }
  const T& top() const { return Top(); }

  void Pop() {
    std::pop_heap(v.begin(), v.end(), Comp());
    v.pop_back();
  }
  void pop() { Pop(); }

  void Clear() { v.clear(); }
  void clear() { Clear(); }

  std::vector<T> v;
};
}  // namespace datastructures

#endif  // SRC_DATASTRUCTURES_VECTOR_PRIORITY_QUEUE_H_
