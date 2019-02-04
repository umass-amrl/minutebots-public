// Copyright 2017 kvedder@umass.edu
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

#ifndef SRC_DATASTRUCTURES_UPDATEABLE_PRIORITY_QUEUE_H_
#define SRC_DATASTRUCTURES_UPDATEABLE_PRIORITY_QUEUE_H_

#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "constants/constants.h"

namespace datastructures {
using DataIndex = std::size_t;
template <class Data, class MetaData>
class UpdateablePriorityQueue {
 public:
  UpdateablePriorityQueue() = default;

  void Push(const Data& data, const MetaData& meta_data) {
    const auto map_search_result = data_to_index_map_.find(data);
    if (map_search_result != data_to_index_map_.end()) {
      const DataIndex& data_index = map_search_result->second;
      // Use the operator< only in order to avoid having to define a second
      // operator.
      if (!(data_store_[data_index].second < meta_data)) {
        data_store_[data_index].second = meta_data;
      }
    }
  }

 private:
  struct QueueWrapper {
    DataIndex data_index;

    explicit QueueWrapper(const DataIndex& data_index)
        : data_index(data_index) {}
    bool operator<(const DataIndex& other) {
      // Compare the meta_data info with eachother.
      return data_store_[data_index].second <
             data_store_[other.data_index].second;
    }
  };

  // Returns which index will be next to pop off of the queue.
  std::priority_queue<QueueWrapper> queue_;
  std::vector<std::pair<Data, MetaData>> data_store_;
  std::unordered_map<Data, DataIndex> data_to_index_map_;
};
}  // namespace datastructures

#endif  // SRC_DATASTRUCTURES_UPDATEABLE_PRIORITY_QUEUE_H_
