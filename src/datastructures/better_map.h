// Copyright 2017 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Forked from https://github.com/kylevedder/BetterMap and relicensed
// by Kyle Vedder (kyle@vedder.io)
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

#ifndef SRC_DATASTRUCTURES_BETTER_MAP_H_
#define SRC_DATASTRUCTURES_BETTER_MAP_H_

#include <memory>
#include <vector>
#include <utility>
#include "constants/constants.h"

namespace datastructures {

template <typename Value> struct OptionalValue {
  bool has_value;
  const Value& val;
};

template <typename Value> struct OptionalValueMutable {
  OptionalValueMutable(bool has_value, Value* val)
      : has_value(has_value), val(val) {}

  OptionalValueMutable(const OptionalValueMutable<Value>& other)
      : has_value(other.has_value), val(other.val) {}

  ~OptionalValueMutable() {}

  OptionalValueMutable<Value>& operator=(
    const OptionalValueMutable<Value>& other) {
    has_value = other.has_value;
    val = other.val;
    return *this;
  }

  bool has_value;
  Value* val;
};

// This map is designed for use with a key of unbounded domain, but will
// hold a finite number of keys. Each key must be unique.
template <typename Value> class BetterMap {
 private:
  unsigned int max_values_;
  unsigned int value_count_;

  struct ValueWrapper {
    bool is_occupied = false;
    unsigned int key;
    Value val;
  };

  std::vector<ValueWrapper> value_store_;

  inline unsigned int HashKey(const unsigned int key) const {
    return key % max_values_;
  }

 public:
  explicit BetterMap(const BetterMap<Value>& other) :
    max_values_(other.max_values_), value_count_(other.value_count_),
    value_store_(other.value_store_) {}

  explicit BetterMap(const unsigned int max_values) : max_values_(max_values),
    value_store_(max_values) {
    value_count_ = 0;
  }

  BetterMap<Value>& operator=(const BetterMap<Value>& other) {
    if (this != &other) {
      max_values_ = other.max_values_;
      value_count_ = other.value_count_;
      value_store_ = other.value_store_;
    }
    return *this;
  }

  ~BetterMap() {}

  BetterMap<Value>(std::initializer_list<std::pair<unsigned int, Value>>
      initializer_list) {
    value_count_ = initializer_list.size();
    max_values_ = initializer_list.size();
    for (const std::pair<unsigned int, Value>& elem : initializer_list) {
      this->Insert(elem.first, elem.second);
    }
  }

  unsigned int CurrentSize() const {
    return value_count_;
  }

  unsigned int MaxSize() const {
    return max_values_;
  }

  // Attempts to insert a key into the map.
  // Will return if the insert was successful.
  bool Insert(unsigned int key, Value val) {
    if (value_count_ >= max_values_) {
      return false;
    }
    unsigned int bucket = HashKey(key);
    // Attempt to read from bucket or all later buckets.
    for (; bucket < max_values_; ++bucket) {
      ValueWrapper& value_wrapper = value_store_[bucket];
      if (!value_wrapper.is_occupied || value_wrapper.key == key) {
        value_wrapper.val = val;
        value_wrapper.key = key;
        value_wrapper.is_occupied = true;
        ++value_count_;
        return true;
      }
    }

    // Attempt another sweep from the beginning.
    for (bucket = 0; bucket < HashKey(key); ++bucket) {
      ValueWrapper& value_wrapper = value_store_[bucket];
      if (!value_wrapper.is_occupied || value_wrapper.key == key) {
        value_wrapper.val = val;
        value_wrapper.key = key;
        value_wrapper.is_occupied = true;
        ++value_count_;
        return true;
      }
    }
    return false;
  }

  const OptionalValue<Value> Read(unsigned int key) const {
    unsigned int bucket = HashKey(key);
    // Attempt to read from bucket or all later buckets.
    for (; bucket < max_values_; ++bucket) {
      const ValueWrapper& value_wrapper = value_store_[bucket];
      if (value_wrapper.is_occupied && value_wrapper.key == key) {
        return OptionalValue<Value>{true, value_wrapper.val};
      }
    }

    // Attempt another sweep from the beginning.
    for (bucket = 0; bucket < HashKey(key); ++bucket) {
      const ValueWrapper& value_wrapper = value_store_[bucket];
      if (value_wrapper.is_occupied && value_wrapper.key == key) {
        return OptionalValue<Value>{true, value_wrapper.val};
      }
    }
    // User should not read from this value...
    return OptionalValue<Value>{false, value_store_[0].val};
  }

  OptionalValueMutable<Value> ReadMutable(unsigned int key) {
    unsigned int bucket = HashKey(key);
    // Attempt to read from bucket or all later buckets.
    for (; bucket < max_values_; ++bucket) {
      ValueWrapper* value_wrapper = &(value_store_[bucket]);
      if (value_wrapper->is_occupied && value_wrapper->key == key) {
        return OptionalValueMutable<Value>(true, &(value_wrapper->val));
      }
    }

    // Attempt another sweep from the beginning.
    for (bucket = 0; bucket < HashKey(key); ++bucket) {
      ValueWrapper* value_wrapper = &(value_store_[bucket]);
      if (value_wrapper->is_occupied && value_wrapper->key == key) {
        return OptionalValueMutable<Value>(true, &(value_wrapper->val));
      }
    }
    ValueWrapper* value_wrapper = &(value_store_[0]);
    // User should not read from this value...
    return OptionalValueMutable<Value>(false, &(value_wrapper->val));
  }

  void Clear() {
    value_count_ = 0;
    value_store_.clear();
  }
};

}  // namespace datastructures

#endif  // SRC_DATASTRUCTURES_BETTER_MAP_H_
