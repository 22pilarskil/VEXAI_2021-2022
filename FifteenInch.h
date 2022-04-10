#ifndef _FifteenInch
#define _FifteenInch
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <atomic>
using namespace pros;

class FifteenInch {
  static Controller controller;
  
  static Motor FL;
  static Motor ML;
  static Motor BL;
  static Motor FR;
  static Motor MR;
  static Motor BR;
  static Motor four_bar;
  
  static Imu IMU;
  
  static void drive(void *ptr);
  static void move_to(void *ptr);
};
#endif
