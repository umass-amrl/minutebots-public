// Copyright 2017 - 2019 kvedder@umass.edu
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

#include <eigen3/Eigen/Core>
#include <glog/logging.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "constants/constants.h"
#include "util/timer.h"
#include "thread_safe/thread_safe_queue.h"
#include "thread_safe/thread_safe_actor.h"
#include "thread_safe/thread_safe_priority_queue.h"

STANDARD_USINGS;
using threadsafe::ThreadSafePriorityQueue;
using threadsafe::ThreadSafeQueue;
using threadsafe::ThreadSafeActor;
using std::thread;

TEST(ThreadSafeTests, HelloWorld) {
  // Should always pass, lets the user know that the test suite is setup.
  ASSERT_TRUE(true);
}

TEST(ThreadSafeTests, BasicQueueTest) {
  ThreadSafeQueue<int> queue;
  for (size_t c = 0; c < 10; ++c) {
    queue.Add(1);
    queue.Add(2);
    queue.Add(3);
    queue.Add(4);
    queue.Add(5);
    std::vector<int> data1;
    queue.ReadAllAndEmpty(&data1);
    for (int i = 0; i < 5; ++i) {
      ASSERT_EQ(data1[i], i+1);
    }
  }
  queue.Shutdown();
}

void AddToQueueInThread(ThreadSafeQueue<int>* queue) {
    queue->Add(1);
    queue->Add(2);
    queue->Add(3);
    queue->Add(4);
    queue->Add(5);
  Sleep(0.2);
  std::vector<int> data1;
  queue->ReadAllAndEmpty(&data1);
  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(data1[i], i + 6);
  }
}

TEST(ThreadSafeTests, BlockQueueTest) {
  ThreadSafeQueue<int> queue;
  for (size_t c = 0; c < 3; ++c) {
    thread t1(&AddToQueueInThread, &queue);
    Sleep(0.1);
    std::vector<int> data1;
    queue.ReadAllAndEmpty(&data1);
    for (int i = 0; i < 5; ++i) {
      ASSERT_EQ(data1[i], i + 1);
    }
    queue.Add(6);
    queue.Add(7);
    queue.Add(8);
    queue.Add(9);
    queue.Add(10);
    t1.join();
  }
  queue.Shutdown();
}


void BlockOnQueueDataInThread(ThreadSafeQueue<int>* queue) {
  std::vector<int> data;
  queue->ReadAllAndEmpty(&data);
}

TEST(ThreadSafeTests, ShutdownQueueTest) {
  for (size_t c = 0; c < 3; ++c) {
    ThreadSafeQueue<int> queue;
    thread t1(&BlockOnQueueDataInThread, &queue);
    Sleep(0.1);
    queue.Shutdown();
    t1.join();
  }
}

TEST(ThreadSafeTests, BasicPriorityQueueTest) {
  ThreadSafePriorityQueue<int, int> queue;
  for (size_t c = 0; c < 10; ++c) {
    queue.Add(1, 1);
    queue.Add(4, 4);
    queue.Add(5, 5);
    queue.Add(3, 3);
    queue.Add(2, 2);

    std::vector<int> data1;
    queue.ReadAllAndEmpty(&data1);

    for (int i = 0; i < 5; ++i) {
      ASSERT_EQ(data1[i], i + 1);
    }
  }
  queue.Shutdown();
}

void AddToPriorityQueueInThread(ThreadSafePriorityQueue<int, int>* queue) {
  queue->Add(2, 2);
  queue->Add(1, 1);
  queue->Add(3, 3);
  queue->Add(5, 5);
  queue->Add(4, 4);
  Sleep(0.2);
  std::vector<int> data1;
  queue->ReadAllAndEmpty(&data1);
  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(data1[i], i + 6);
  }
}

TEST(ThreadSafeTests, BlockPriorityQueueTest) {
  ThreadSafePriorityQueue<int, int> queue;
  for (size_t c = 0; c < 3; ++c) {
    thread t1(&AddToPriorityQueueInThread, &queue);
    Sleep(0.1);
    std::vector<int> data1;
    queue.ReadAllAndEmpty(&data1);
    for (int i = 0; i < 5; ++i) {
      ASSERT_EQ(data1[i], i + 1);
    }
    queue.Add(10, 10);
    queue.Add(6, 6);
    queue.Add(8, 8);
    queue.Add(9, 9);
    queue.Add(7, 7);
    t1.join();
  }
  queue.Shutdown();
}


void BlockPriorityQueueDataInThread(ThreadSafePriorityQueue<int, int>* queue) {
  std::vector<int> data;
  queue->ReadAllAndEmpty(&data);
}

TEST(ThreadSafeTests, ShutdownPriorityQueueTest) {
  for (size_t c = 0; c < 3; ++c) {
    ThreadSafePriorityQueue<int, int> queue;
    thread t1(&BlockPriorityQueueDataInThread, &queue);
    Sleep(0.1);
    queue.Shutdown();
    t1.join();
  }
}

TEST(ThreadSafeTests, BasicActorTest) {
  ThreadSafeActor<int> actor(0);
  actor.Write(1);
  int item = 0;
  actor.ReadOrDefault(&item);
  ASSERT_EQ(item, 1);
  actor.Shutdown();
}

void SetActorInThread(ThreadSafeActor<int>* actor) {
  actor->Write(1);
}

TEST(ThreadSafeTests, SetActorInThreadTest) {
  ThreadSafeActor<int> actor(0);
  thread t1(&SetActorInThread, &actor);
  Sleep(0.1);
  int item = 0;
  actor.ReadOrDefault(&item);
  ASSERT_EQ(item, 1);
  actor.Shutdown();
  t1.join();
}
