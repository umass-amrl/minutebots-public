// Copyright 2017 - 2018 kvedder@umass.edu
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
#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "eigen3/Eigen/Core"

#include "constants/constants.h"
#include "datastructures/bounded_queue.h"
#include "datastructures/dense_array.h"
#include "datastructures/radixheap.h"
#include "datastructures/vector_priority_queue.h"
#include "math/poses_2d.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

STANDARD_USINGS;
using datastructures::BoundedQueue;
using datastructures::DenseArray;
using datastructures::RadixHeap;

TEST(DataTests, RadixHeapArbitraryDatatype) {
  static constexpr int max_size = 1000;
  RadixHeap<Vector2f> rh(max_size);

  const Vector2f expected_min(20, 20);
  rh.Insert({10, 10}, 100, 20);
  rh.Insert(expected_min, 400, 10);

  Vector2f min = rh.DeleteMin();
  ASSERT_EQ(min, expected_min);
}

TEST(DataTests, BoundVectorIterTest) {
  BoundedQueue<int> queue(10);
  vector<int> vec;
  for (int i = 0; i < 3; ++i) {
    vec.push_back(i + 100);
    queue.Add(i + 100);
  }

  for (int i = 0; i < 3; ++i) {
    const int underlying_read = queue.UnderlyingRead(i);
    EXPECT_EQ(underlying_read, vec[i]);
  }

  for (size_t j = 0; j < vec.size(); ++j) {
    LOG(INFO) << "Index " << j;
    const int queue_val = queue[j];
    LOG(INFO) << "queue: " << queue_val;
    LOG(INFO) << "vec: " << vec[j];
    EXPECT_EQ(queue_val, vec[j]);
  }
}

TEST(DataTests, BoundVectorIterTestLargeDataset) {
  BoundedQueue<int> queue(10);
  vector<int> vec;
  for (int i = 0; i < 13; ++i) {
    if (i >= 3) {
      vec.push_back(i + 100);
    }
    queue.Add(i + 100);
  }

  for (size_t j = 0; j < vec.size(); ++j) {
    LOG(INFO) << "Index " << j;
    const int queue_val = queue[j];
    LOG(INFO) << "queue: " << queue_val;
    LOG(INFO) << "vec: " << vec[j];
    EXPECT_EQ(queue_val, vec[j]);
  }
}

TEST(DataTests, BoundVectorIterTestLargerDataset) {
  BoundedQueue<int> queue(10);
  vector<int> vec;
  for (int i = 0; i < 23; ++i) {
    if (i >= 13) {
      vec.push_back(i + 100);
    }
    queue.Add(i + 100);
  }

  for (size_t j = 0; j < vec.size(); ++j) {
    LOG(INFO) << "Index " << j;
    const int queue_val = queue[j];
    LOG(INFO) << "queue: " << queue_val;
    LOG(INFO) << "vec: " << vec[j];
    EXPECT_EQ(queue_val, vec[j]);
  }
}

TEST(DataTests, BoundTest) {
  BoundedQueue<int> queue(10);

  EXPECT_EQ(queue.Size(), 0u);

  queue.Add(0);

  EXPECT_EQ(queue.First(), 0);
  LOG(INFO) << "First\n";
  EXPECT_EQ(queue.Last(), 0);
  EXPECT_EQ(queue.Size(), 1u);

  for (size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(queue.Size(), i + 1);
    queue.Add(static_cast<int>(i + 1));
    EXPECT_EQ(queue.Size(), i + 2);
  }
  EXPECT_EQ(queue.First(), 9);
  EXPECT_EQ(queue.Last(), 0);
  EXPECT_EQ(queue.Size(), 10u);

  for (int i = 10; i <= 19; ++i) {
    EXPECT_EQ(queue.Size(), 10u);
    queue.Add(i);
    EXPECT_EQ(queue.Size(), 10u);
  }

  EXPECT_EQ(queue.First(), 19);
  EXPECT_EQ(queue.Last(), 10);
  EXPECT_EQ(queue.Size(), 10u);
}

TEST(DataTests, DenseArray) {
  DenseArray<std::string, 6> array;

  array.InsertBack("hello");
  array.InsertBack("world");

  EXPECT_EQ(array.GetElementCount(), 2u);
  EXPECT_EQ(array.Get(0), "hello");
  EXPECT_EQ(array.Get(1), "world");

  array.DeleteAndShiftLeft(0);

  EXPECT_EQ(array.GetElementCount(), 1u);
  EXPECT_EQ(array.Get(0), "world");

  array.InsertAndShiftRight(0, "hi");

  EXPECT_EQ(array.GetElementCount(), 2u);
  EXPECT_EQ(array.Get(0), "hi");
  EXPECT_EQ(array.Get(1), "world");

  array.InsertAndShiftRight(0, "sup fam");

  array.InsertAndShiftRight(0, "what's good?");

  EXPECT_EQ(array.GetElementCount(), 4u);
  EXPECT_EQ(array.Get(0), "what's good?");
  EXPECT_EQ(array.Get(1), "sup fam");
  EXPECT_EQ(array.Get(2), "hi");
  EXPECT_EQ(array.Get(3), "world");

  array.InsertBack("it's lit");

  EXPECT_EQ(array.GetElementCount(), 5u);
  EXPECT_EQ(array.Get(0), "what's good?");
  EXPECT_EQ(array.Get(1), "sup fam");
  EXPECT_EQ(array.Get(2), "hi");
  EXPECT_EQ(array.Get(3), "world");
  EXPECT_EQ(array.Get(4), "it's lit");

  std::vector<std::string> comp_data = {"what's good?", "sup fam", "hi",
                                        "world", "it's lit"};
  size_t i = 0;
  for (const std::string& s : array) {
    EXPECT_EQ(s, comp_data[i]);
    EXPECT_EQ(s, array.Get(i));
    i++;
  }

  // Checks that running the iterator more than once works as expected.
  i = 0;
  for (const std::string& s : array) {
    EXPECT_EQ(s, comp_data[i]);
    EXPECT_EQ(s, array.Get(i));
    i++;
  }

  array.DeleteAndShiftLeft(0);
  array.DeleteAndShiftLeft(0);

  comp_data = {"hi", "world", "it's lit"};

  i = 0;
  for (const std::string& s : array) {
    EXPECT_EQ(s, comp_data[i]);
    EXPECT_EQ(s, array.Get(i));
    i++;
  }
}

TEST(Datatests, DenseArray2) {
  DenseArray<std::string, 6> array;
  for (size_t i = 0; i < 1000; ++i) {
    array.InsertBack("sorry about the cringy strings");
    array.DeleteAndShiftLeft(0);
  }
  EXPECT_EQ(array.GetElementCount(), 0ul);
}

TEST(Datatests, DenseArray3) {
  DenseArray<std::string, 6> array;
  array.InsertBack("base string");
  for (size_t i = 0; i < 1000; ++i) {
    array.InsertBack("sorry about the cringy strings");
    array.DeleteAndShiftLeft(1);
  }
  EXPECT_EQ(array.GetElementCount(), 1ul);
  EXPECT_EQ(array.Get(0), "base string");
}

TEST(VectorPriorityQueue, SortTest) {
  datastructures::VectorPriorityQueue<int> vpq;

  std::vector<int> raw_start_data = {3, 5, 3, 9, 0, 8, 1, 3, 8, 7};

  for (const auto& e : raw_start_data) {
    vpq.Push(e);
  }

  for (auto& e : *vpq.GetMutableVector()) {
    e += 2;
  }

  std::sort(raw_start_data.begin(), raw_start_data.end());
  std::reverse(raw_start_data.begin(), raw_start_data.end());

  for (const auto& e : raw_start_data) {
    EXPECT_EQ(vpq.Top(), e + 2);
    EXPECT_FALSE(vpq.Empty());
    vpq.Pop();
  }
  EXPECT_TRUE(vpq.Empty());
}

TEST(VectorPriorityQueue, AppendData) {
  const std::vector<int> raw_start_data = {3, 5, 3, 9, 0, 8, 9, 3, 8, 7};
  const std::vector<int> raw_start_data2 = {3, 5, 3, 10, 0, 8, 1, 3, 8, 7};
  datastructures::VectorPriorityQueue<int> vpq;
  for (const auto& e : raw_start_data) {
    vpq.Push(e);
  }

  datastructures::VectorPriorityQueue<int> vpq2(raw_start_data2);

  vpq.MergeOther(vpq2);
  vpq.RebuildHeap();

  EXPECT_EQ(10, vpq.Top());
}
