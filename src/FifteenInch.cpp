#include "FifteenInch.h"
#include <map>
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
using namespace pros;
using namespace std;

Controller FifteenInch::master(E_CONTROLLER_MASTER);

Motor FifteenInch::FL(17, true);
Motor FifteenInch::ML(18, true);
Motor FifteenInch::BL(19, true);
Motor FifteenInch::FR(13);
Motor FifteenInch::MR(12);
Motor FifteenInch::BR(11);
Motor FifteenInch::four_bar(20, true);

Imu FifteenInch::IMU(15);
Rotation FifteenInch::left_dead_wheel(21);
Rotation FifteenInch::right_dead_wheel(14);

void FifteenInch::drive(void *ptr){
  
}

void FifteenInch::move_to(void *ptr){
  
}
