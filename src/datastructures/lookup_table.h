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
#ifndef SRC_DATASTRUCTURES_LOOKUP_TABLE_H_
#define SRC_DATASTRUCTURES_LOOKUP_TABLE_H_

#include <algorithm>
#include <vector>

#include "constants/constants.h"

namespace datastructures {

template <typename Payload>
class DynamicAllocLookupTable {
 public:
  DynamicAllocLookupTable() = delete;
  DynamicAllocLookupTable(const size_t max_key, const Payload& default_value)
      : table_(max_key, default_value), default_value_(default_value) {}

  void ResetTable() { std::fill(table_.begin(), table_.end(), default_value_); }

  Payload Read(const size_t index) const {
    NP_CHECK(index < table_.size());
    return table_[index];
  }

  void Overwrite(const size_t index, const Payload& payload) {
    NP_CHECK(index < table_.size());
    table_[index] = payload;
  }

  void Add(const size_t index, const Payload& payload) {
    NP_CHECK(index < table_.size());
    table_[index] += payload;
  }

  void Subtract(const size_t index, const Payload& payload) {
    NP_CHECK(index < table_.size());
    table_[index] -= payload;
  }

  void Or(const size_t index, const Payload& payload) {
    NP_CHECK(index < table_.size());
    table_[index] |= payload;
  }

  void And(const size_t index, const Payload& payload) {
    NP_CHECK(index < table_.size());
    table_[index] &= payload;
  }

  inline size_t Size() const { return size(); }

  inline size_t size() const { return table_.size(); }

  const std::vector<Payload>& GetTable() const { return table_; }

 private:
  std::vector<Payload> table_;
  const Payload default_value_;
};

}  // namespace datastructures

#endif  // SRC_DATASTRUCTURES_LOOKUP_TABLE_H_
