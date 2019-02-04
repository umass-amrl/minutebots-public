// Copyright 2017 joydeepb@cs.umass.edu, kvedder@umass.edu
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

#ifndef SRC_DATASTRUCTURES_RADIXHEAP_H_
#define SRC_DATASTRUCTURES_RADIXHEAP_H_

#include "constants/constants.h"

namespace datastructures {
template <class Data>

class RadixHeap {
 public:
  RadixHeap() = delete;

  // Determines the upper bound on the domain of indices that the heap can hold.
  // This is important as fix the size of the underlying radix array, so access
  // beyond the given max will be erroronious.
  explicit RadixHeap(int max_data_index)
      : n_buckets_(static_cast<int>(std::ceil(std::log2(kMaxKey + 1)) + 2)),
        item_count_(0) {
    // Allocate node lookup array (indexed by item no).
    nodes_ = new RadixHeapNode *[max_data_index];
    for (int i = 0; i < max_data_index; ++i) {
      nodes_[i] = nullptr;
    }

    // Allocate and initialise buckets.
    RadixHeapNode blank_node;
    bucket_headers_ = new RadixHeapNode[n_buckets_ + 1];
    for (int i = 0; i <= n_buckets_; i++) {
      bucket_headers_[i] = blank_node;
      bucket_headers_[i].next = &bucket_headers_[i];
      bucket_headers_[i].prev = &bucket_headers_[i];
    }

    // Allocate and initialse upper-limits of buckets.
    upper_limits_ = new int[n_buckets_ + 1];
    upper_limits_[0] = -1;
    int l = 1;
    for (int i = 1; i <= n_buckets_; i++) {
      upper_limits_[i] = l - 1;
      l *= 2;
    }
    upper_limits_[n_buckets_] = max_data_index * kMaxKey + 1;
  }

  ~RadixHeap() {
    while (NItems() > 0) {
      DeleteMin();
    }
    delete[] nodes_;
    delete[] bucket_headers_;
    delete[] upper_limits_;
  }

  void Clear() {
    while (NItems() > 0) {
      DeleteMin();
    }
  }

  Data DeleteMin() {
    // If bucket 1 is nonempty, return any of its nodes as the minimum.
    if (bucket_headers_[1].next != &bucket_headers_[1]) {
      RadixHeapNode *min_node = bucket_headers_[1].next;
      RemoveNode(min_node);
      Data min_data = min_node->data;
      int min_data_index = min_node->data_index;
      nodes_[min_data_index] = nullptr;
      delete min_node;
      item_count_--;
      return min_data;
    }

    // Find i such that bucket i is the smallest nonempty bucket.
    int i = 2;
    while (bucket_headers_[i].next == &bucket_headers_[i]) i++;

    // Find and remove the minimum node from bucket i.
    RadixHeapNode *header = &bucket_headers_[i];
    RadixHeapNode *min_node = bucket_headers_[i].next;
    int min_key = min_node->key;
    RadixHeapNode *node = min_node->next;
    while (node != header) {
      if (node->key < min_key) {
        min_node = node;
        min_key = node->key;
      }
      node = node->next;
    }
    RemoveNode(min_node);

    // Recalulate upper bounds on empty buckets.
    upper_limits_[0] = min_key - 1;
    upper_limits_[1] = min_key;
    int l = 1;
    int s = min_key;
    int uMax = upper_limits_[i];
    for (int j = 2; j < i; j++) {
      s += l;
      upper_limits_[j] = s < uMax ? s : uMax;
      l *= 2;
    }

    // Every vertex in u[i] can now be moved to the empty lower buckets.
    // This is gauranteed since the condition u[i] = u[i-1] must hold.

    // Place nodes from bucket i into lower buckets.
    RadixHeapNode *next_node = header->next;
    while (next_node != header) {
      node = next_node;
      next_node = next_node->next;
      PlaceNode(i - 1, node);
    }

    // Bucket i can now be marked as empty.
    bucket_headers_[i].next = bucket_headers_[i].prev = &bucket_headers_[i];

    // Delete the minimum node and return the corresponding item.
    if (kRadixHeapDebug) {
      LOG(INFO) << "performed delete-min " << min_node->data << "("
                << min_node->key << ")";
      Dump();
    }
    Data min_data = min_node->data;
    int min_data_index = min_node->data_index;
    nodes_[min_data_index] = nullptr;
    delete min_node;
    item_count_--;
    return min_data;
  }

  void Insert(Data item, int item_index, int64_t key_weighting) {
    RadixHeapNode *new_node =
        new RadixHeapNode(item, item_index, key_weighting);
    nodes_[item_index] = new_node;
    PlaceNode(n_buckets_, new_node);
    item_count_++;
    if (kRadixHeapDebug) {
      LOG(INFO) << "performed insert " << item << " at index " << item_index
                << "(" << key_weighting << ")";
      Dump();
    }
  }
  void DecreaseKey(int item_index, int64_t key_weighting) {
    RadixHeapNode *node = nodes_[item_index];
    RemoveNode(node);
    node->key = key_weighting;
    PlaceNode(node->bucket, node);
    if (kRadixHeapDebug) {
      LOG(INFO) << "performed decrease-key (" << key_weighting << ") on item "
                << node->data;
      Dump();
    }
  }
  int NItems() const { return item_count_; }

  int64_t NComps() const { return comp_count_; }

  void Dump() const {
    int i = n_buckets_;
    for (; i > 0 && bucket_headers_[i].next == &bucket_headers_[i]; i--) {
    }

    do {
      LOG(INFO) << "bucket " << i << "[" << upper_limits_[i] << "]:  ";
      RadixHeapNode *header = &bucket_headers_[i];
      RadixHeapNode *node = header->next;
      while (node != header) {
        LOG(INFO) << node->data << "(" << node->key << "), ";
        if (node->key > upper_limits_[i] || node->key <= upper_limits_[i - 1]) {
          LOG(INFO) << std::endl
                    << " error: node in wrong bucket" << std::endl
                    << " ";
          exit(1);
        }
        node = node->next;
      }
      LOG(INFO) << std::endl;
      i--;
    } while (i >= 0);
  }

 private:
  class RadixHeapNode {
   public:
    Data data;
    int data_index;
    int key;
    int bucket;
    RadixHeapNode *next;
    RadixHeapNode *prev;

    RadixHeapNode()
        : data(),
          data_index(-1),
          key(-1),
          bucket(-1),
          next(nullptr),
          prev(nullptr) {}

    RadixHeapNode(const Data data, const int data_index, const int key)
        : data(data), data_index(data_index), key(key) {}

    ~RadixHeapNode() = default;
  };

  void PlaceNode(int start_bucket, RadixHeapNode *node) {
    int key = node->key;
    int i = start_bucket;
    do {
      i--;
    } while (upper_limits_[i] >= key);
    InsertNode(i + 1, node);
  }
  void InsertNode(int i, RadixHeapNode *node) {
    /* link the node into bucket i */
    node->bucket = i;
    RadixHeapNode *tail_node = &bucket_headers_[i];
    RadixHeapNode *prev_node = tail_node->prev;
    node->next = tail_node;
    tail_node->prev = node;
    node->prev = prev_node;
    prev_node->next = node;
  }
  void RemoveNode(RadixHeapNode *node) {
    // Unlink the node from its bucket.
    node->prev->next = node->next;
    node->next->prev = node->prev;
  }

  static constexpr int kMaxKey = 500000;
  static constexpr bool kRadixHeapDebug = false;
  RadixHeapNode **nodes_;
  RadixHeapNode *bucket_headers_;
  int *upper_limits_;

  int n_buckets_;

  int item_count_;
  int comp_count_;
};

}  // namespace datastructures

#endif  // SRC_DATASTRUCTURES_RADIXHEAP_H_
