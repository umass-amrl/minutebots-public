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

#ifndef SRC_DATASTRUCTURES_TRANSACTED_MULTI_VECTOR_PRIORITY_QUEUE_H_
#define SRC_DATASTRUCTURES_TRANSACTED_MULTI_VECTOR_PRIORITY_QUEUE_H_

#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "constants/constants.h"

namespace datastructures {
template <class T>
class TransactedMultiVectorPriorityQueue {
 private:
  std::vector<std::vector<T>> v_vec;
  size_t push_pops_;
  size_t reorders_;

  std::vector<T>* GetCurrentAppendVector() {
    NP_CHECK(!v_vec.empty());
    return &(v_vec[v_vec.size() - 1]);
  }

 public:
  TransactedMultiVectorPriorityQueue() : v_vec(), push_pops_(0), reorders_(0) {
    v_vec.push_back({});
  }
  explicit TransactedMultiVectorPriorityQueue(const std::vector<T>& v_arg)
      : v_vec(), push_pops_(0), reorders_(0) {
    v_vec.push_back(v_arg);
    ++reorders_;
    std::vector<T>* v = GetCurrentAppendVector();
    std::make_heap(v->begin(), v->end());
  }

  TransactedMultiVectorPriorityQueue(
      const TransactedMultiVectorPriorityQueue<T>& v1,
      const TransactedMultiVectorPriorityQueue<T>& v2)
      : TransactedMultiVectorPriorityQueue(v1.GetVector(), v2.GetVector()) {}

  TransactedMultiVectorPriorityQueue(const std::vector<T>& v1,
                                     const std::vector<T>& v2)
      : v_vec(), push_pops_(0), reorders_(0) {
    v_vec.push_back(v1);
    std::vector<T>* v = GetCurrentAppendVector();
    ++reorders_;
    v->insert(v->end(), v2.begin(), v2.end());
    std::make_heap(v->begin(), v->end());
  }

  std::vector<std::vector<T>>* GetMutableVectors() { return &v_vec; }

  void Rebuild() {
    ++reorders_;
    for (std::vector<T>& v : v_vec) {
      std::make_heap(v.begin(), v.end());
    }
  }

  void Push(const T& e) {
    static constexpr float kGrowthRate = 1.05f;
    ++push_pops_;
    std::vector<T>* v = GetCurrentAppendVector();
    try {
      v->push_back(e);
    } catch (const std::bad_alloc& exception) {
      const size_t new_reserve_size =
          static_cast<size_t>(static_cast<float>(v->capacity()) * kGrowthRate);
      LOG(INFO) << "Reserving from " << v->capacity() * sizeof(T) << " to "
                << new_reserve_size * sizeof(T);
      try {
        v->reserve(new_reserve_size);
      } catch (const std::bad_alloc& exception) {
        LOG(INFO) << "Adding new vector";
        v_vec.push_back({});
        v = GetCurrentAppendVector();
      }
      v->push_back(e);
    }
    std::push_heap(v->begin(), v->end());
  }
  void push(const T& e) { Push(e); }

  bool Empty() const {
    for (const auto& v : v_vec) {
      if (!v.empty()) {
        return false;
      }
    }
    return true;
  }
  inline bool empty() const { return Empty(); }

  size_t Size() {
    size_t size = 0;
    for (auto& v : v_vec) {
      size += v.size();
    }
    return size;
  }
  inline size_t size() { return Size(); }

  const std::pair<T const*, size_t> TopAndIndex() const {
    if (!kProduction && Empty()) {
      LOG(FATAL) << "Cannot get the top element of an empty queue";
    }

    NP_CHECK(!v_vec[0].empty());
    T const* smallest = &(v_vec[0][0]);
    size_t smallest_idx = 0;
    for (size_t i = 1; i < v_vec.size(); ++i) {
      if (v_vec[i].empty()) {
        continue;
      }
      if (v_vec[i][0] < *smallest) {
        smallest = &(v_vec[i][0]);
        smallest_idx = i;
      }
    }
    return {smallest, smallest_idx};
  }

  const T& Top() const { return *(TopAndIndex().first); }
  const T& top() const { return Top(); }

  void Pop() {
    ++push_pops_;
    const size_t i = TopAndIndex().second;
    NP_CHECK(i < v_vec.size());
    std::pop_heap(v_vec[i].begin(), v_vec[i].end());
    v_vec[i].pop_back();
  }
  void pop() { Pop(); }

  void Clear() {
    v_vec.clear();
    v_vec.push_back({});
  }
  inline void clear() { Clear(); }

  size_t GetPushPopsCounts() const { return push_pops_; }
  size_t GetReorderCounts() const { return reorders_; }
  void ResetCounts() {
    push_pops_ = 0;
    reorders_ = 0;
  }
};
}  // namespace datastructures

#endif  // SRC_DATASTRUCTURES_TRANSACTED_MULTI_VECTOR_PRIORITY_QUEUE_H_
