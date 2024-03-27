#include <stdio.h>
#include <stdint.h>

#include "simple_queue.h"

int main() {
  // Construct the priority queue, to use uint64_t to represent node ID, and float as the priority type.
  SimpleQueue<uint64_t, float, float> queue;
  // Check if the queue is empty
  printf("Empty?: %d\n", queue.Empty());
  // Insert the node with ID 42 with priority 2.34 into the queue:
  queue.Push(42, 2.43, 3.0);
  // Insert the node with ID 43 with priority 5.1 into the queue:
  queue.Push(43, 5.1, 0.0);
  // Update the node with ID 42 with priority 2.1 into the queue:
  // Get the priority element, this will return ID 43, since it has the lower sum of the two priority metrics
  uint64_t next_node = queue.Pop();
  printf("Next: %lu\n", next_node);
  // should be 42
  next_node = queue.Pop();
  printf("Next: %lu\n", next_node);
  // test case where there one has more "known data"
  queue.Push(44, 7.9, 1.1);
  queue.Push(42, 8.0, 1.0); // this should come first
  // Check if the queue is empty
  printf("Empty?: %d\n", queue.Empty());
  // Get the priority element, this will return ID 42, since it has the better priority (2.1 < 5.1)
  next_node = queue.Pop();
  printf("Next: %lu\n", next_node);
}
