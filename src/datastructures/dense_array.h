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
#ifndef SRC_DATASTRUCTURES_DENSE_ARRAY_H_
#define SRC_DATASTRUCTURES_DENSE_ARRAY_H_

#include <algorithm>
#include <array>
#include <utility>

#include "constants/constants.h"

namespace datastructures {

// Dense array supporting checks on current element count and deletions of
// elements in the array.
template <typename T, size_t N>
class DenseArray {
 private:
  size_t current_size_ = 0;
  std::array<T, N> array_;

  class DenseArrayIter {
   public:
    DenseArrayIter() = delete;
    DenseArrayIter(DenseArray<T, N>* dense_array, size_t pos)
        : dense_array_(dense_array), pos_(pos) {
      NP_CHECK(pos_ <= N);
    }

    // The following three methods together form the basis of an iterator which
    // allows us to iterate over all of the selected obstacles.
    bool operator!=(const DenseArrayIter& other) const {
      return pos_ != other.pos_;
    }
    T& operator*() const { return *(dense_array_->GetMutable(pos_)); }
    DenseArrayIter& operator++() {
      ++pos_;
      NP_CHECK(pos_ <= N);
      return *this;
    }

    DenseArrayIter& operator+(const int i) {
      pos_ += i;
      NP_CHECK(pos_ <= N);
      return *this;
    }

    DenseArrayIter& operator+=(const int i) {
      pos_ += i;
      NP_CHECK(pos_ <= N);
      return *this;
    }

   private:
    DenseArray<T, N>* dense_array_;
    size_t pos_;
  };

  class ConstDenseArrayIter {
   public:
    ConstDenseArrayIter() = delete;
    ConstDenseArrayIter(const DenseArray<T, N>& dense_array, size_t pos)
        : dense_array_(dense_array), pos_(pos) {}

    // The following three methods together form the basis of an iterator which
    // allows us to iterate over all of the selected obstacles.
    bool operator!=(const ConstDenseArrayIter& other) const {
      return pos_ != other.pos_;
    }
    const T& operator*() const { return dense_array_.Get(pos_); }
    ConstDenseArrayIter& operator++() {
      ++pos_;
      return *this;
    }

   private:
    const DenseArray<T, N>& dense_array_;
    size_t pos_;
  };

 public:
  DenseArray() = default;
  DenseArray(const T& default_data, const size_t k) {
    NP_CHECK(k <= N);
    std::fill(array_.begin(), array_.begin() + k, default_data);
    current_size_ = k;
  }
  DenseArray(const DenseArray<T, N>& other) = default;
  DenseArray(DenseArray<T, N>&& other) = default;
  ~DenseArray() = default;

  DenseArray& operator=(const DenseArray<T, N>& other) = default;
  DenseArray& operator=(DenseArray<T, N>&& other) = default;

  bool operator==(const DenseArray<T, N>& other) const {
    if (GetElementCount() != other.GetElementCount()) {
      return false;
    }
    for (size_t i = 0; i < GetElementCount(); ++i) {
      if (Get(i) != other.Get(i)) {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const DenseArray<T, N>& other) const {
    return !((*this) == other);
  }

  void operator+=(const DenseArray<T, N>& other) {
    NP_CHECK(this->size() == other.size());
    for (size_t i = 0; i < other.size(); ++i) {
      array_[i] += other.at(i);
    }
  }

  DenseArray<T, N> operator+(const DenseArray<T, N>& other) const {
    NP_CHECK(this->size() == other.size());
    DenseArray<T, N> result;
    for (size_t i = 0; i < other.size(); ++i) {
      result.push_back(this->at(i) + other.at(i));
    }
    return result;
  }

  // Gets the count of the current number of elements stored in this
  // datastructure.
  inline size_t GetElementCount() const { return current_size_; }

  // STL compatability.
  inline size_t size() const { return GetElementCount(); }

  inline bool Empty() const { return (current_size_ <= 0); }

  inline bool empty() const { return Empty(); }

  // Deletes the data at the given index and shifts the data to the right left
  // one index.
  void DeleteAndShiftLeft(const size_t index) {
    NP_CHECK(index < current_size_);
    for (size_t i = index + 1; i < current_size_; ++i) {
      array_[i - 1] = array_[i];
    }
    --current_size_;
  }

  // Inserts the data at the given index and shifts the data to the right right
  // one index.
  void InsertAndShiftRight(const size_t index, const T& data) {
    NP_CHECK(index < N);
    NP_CHECK(current_size_ + 1 < N);
    // int casting is to help with possible wrap around issues when nearing
    // zero.
    for (int i = static_cast<int>(current_size_ - 1);
         i >= static_cast<int>(index); --i) {
      array_[i + 1] = array_[i];
    }
    // Insert given data into the freshly made slot.
    array_[index] = data;
    ++current_size_;
  }

  // Insertion into the first free element in the array.
  void InsertBack(const T& data) {
    NP_CHECK_MSG(current_size_ < N,
                 "Attempt to insert at back failed. Current size: "
                     << current_size_ << " Array size: " << N
                     << " Size after insert: " << (current_size_ + 1));
    array_[current_size_] = data;
    ++current_size_;
  }

  // STL compatability.
  inline void push_back(const T& data) { InsertBack(data); }

  // Gets the element at the given index.
  const T& Get(const size_t index) const {
    NP_CHECK(index < N);
    return array_[index];
  }

  // STL compatability. Does tighter bounds checking, and thus is preferred to
  // Get().
  inline const T& at(const size_t index) const {
    NP_CHECK(index < current_size_);
    return Get(index);
  }

  // Gets a mutable pointer element at the given index.
  T* GetMutable(const size_t index) {
    NP_CHECK(index < N);
    return &(array_[index]);
  }

  // Gets the element at the given index.
  // Allows for setting of indices outside of the currently set range, but not
  // outside of the array.
  void Set(const size_t index, const T& value) {
    NP_CHECK(index < N);
    array_[index] = value;
  }

  // Returns a pointer to the underlying std::array. Use with caution.
  std::array<T, N>* GetMutableUnderlyingArray() { return &array_; }

  // Forces a new size to be set in the dense array. Use with caution.
  inline void ForceSetSize(const size_t new_size) {
    NP_CHECK(new_size <= N);
    current_size_ = new_size;
  }

  bool Contains(const T& q) const {
    for (const auto& e : *this) {
      if (e == q) {
        return true;
      }
    }
    return false;
  }

  void SetUnion(const DenseArray<T, N>& other) {
    for (const T& e : other) {
      if (!this->Contains(e)) {
        this->push_back(e);
      }
    }
  }

  DenseArrayIter begin() { return DenseArrayIter(this, 0u); }

  DenseArrayIter end() { return DenseArrayIter(this, current_size_); }

  ConstDenseArrayIter begin() const { return ConstDenseArrayIter(*this, 0u); }

  ConstDenseArrayIter end() const {
    return ConstDenseArrayIter(*this, current_size_);
  }

  inline void Clear() { current_size_ = 0; }

  // STL compatability.
  inline void clear() { Clear(); }

  inline constexpr size_t MaxSize() const { return N; }

  T MinElement() const {
    NP_CHECK(!empty());
    if (empty()) {
      return {};
    }
    T min = array_[0];
    for (const T& e : *this) {
      min = std::min(e, min);
    }
    return min;
  }

  T MaxElement() const {
    NP_CHECK(!empty());
    T max = array_[0];
    for (const T& e : *this) {
      max = std::max(e, max);
    }
    return max;
  }

  T Sum() const {
    T sum = 0;
    for (const T& e : *this) {
      sum += e;
    }
    return sum;
  }
};

}  // namespace datastructures

#endif  // SRC_DATASTRUCTURES_DENSE_ARRAY_H_
