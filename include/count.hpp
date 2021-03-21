#ifndef _COUNT_H_
#define _COUNT_H_
#include <vector>
#include <array>
#include <string>

// shooter
extern struct OUT {
  int down;
  int up;
  bool wentDownShot;
} OUT_t;
extern OUT out;

// running sum of balls in the robot
// cross check running sums
extern struct COUNT {
  int shotOut;
} COUNT_t;
extern COUNT count;

extern std::array<bool, 2> filled;
extern std::string color;

/*
* @task: State Machine for Counting Balls
*/
extern void countTask(void* param);

#endif
