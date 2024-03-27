// Copyright (c) 2018 joydeepb@cs.umass.edu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <deque>
#include <utility>
#include <tuple>

#ifndef MUTABLE_QUEUE
#define MUTABLE_QUEUE

using std::deque;
// uses first & second
// using std::pair;
// using std::make_pair;
// uses std::get<index>(tuple_name) 
using std::tuple;
using std::make_tuple;

template<class Value, class Cost, class Heuristic>
class SimpleQueue {
 private:
  public:
  // Insert a new value, with the specified Cost. If the value
  // already exists, its Cost is updated.
  void Push(const Value& v, const Cost& p, const Heuristic& h) {
    for (auto& x : values_) {
      // If the value already exists, update its Cost, re-sort the Cost
      // queue, and return.
      if (std::get<0>(x) == v) {
        std::get<1>(x) = p;
        std::get<2>(x) = h;
        Sort();
        return;
      }
    }
    // Find where this value should go, and insert it there.
    for (size_t i = 0; i < values_.size(); ++i) {
      if (std::get<1>(values_[i]) + std::get<2>(values_[i]) > p + h) {
        values_.insert(values_.begin() + i, make_tuple(v, p, h));
        return;
      }
    }
    values_.insert(values_.end(), make_tuple(v, p, h));
  }

  // Sorts the priorities.
  void Sort() {
    static const auto comparator = 
        [](const tuple<Value, Cost, Heuristic>& v1, const tuple<Value, Cost, Heuristic>& v2) {
      auto v1_cost = std::get<1>(v1) + std::get<2>(v1);
      auto v2_cost = std::get<1>(v2) + std::get<2>(v2);
      return (v1_cost != v2_cost) ? (v1_cost > v2_cost) : (std::get<2>(v1) > std::get<2>(v2));
    };
    sort(values_.begin(), values_.end(), comparator);
  }

  // Retreive the value with the highest Cost.
  Value Pop() {
    if (values_.empty()) {
      fprintf(stderr, "ERROR: Pop() called on an empty queue!\n");
      exit(1);
    }
    Sort();
    const Value v = std::get<0>(values_.back());
    values_.resize(values_.size() - 1);
    return v;
  }

  // Returns true iff the Cost queue is empty.
  bool Empty() {
    return values_.empty();
  }

  // Returns true iff the provided value is already on the queue.
  bool Exists(const Value& v) {
    for (const auto& x : values_) {
      if (std::get<0>(x) == v) return true;
    }
    return false;
  }

  private:
  deque<tuple<Value, Cost, Heuristic> > values_;
};

#endif  // MUTABLE_QUEUE
