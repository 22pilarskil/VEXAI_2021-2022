
#include "main.h"
#include "FifteenInch.h"
#include "system/json.hpp"
#include "system/Serial.h"
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

void FifteenInch::tank_drive(int power, int turn)
{
  if(turn > 0 && power >= 0)
  {
    int right_side = ((power - turn) > 127) ? 127 : (power - turn);
    int left_side = (-(power + turn) < -127) ? -127 : -(power + turn);
  }
  else if(turn < 0 && power >= 0)
  {
    int right_side = ((power - turn) > 127) ? 127 : (power - turn);
    int left_side = power + turn;
  }
  else if(turn > 0 && power <= 0)
  {
    int right_side = power + turn;
    int left_side = ((turn - power) > 127) ? 127 : (turn - power);
  }
  else if(turn < 0 && power <= 0)
  {
    int right_side = ((turn + power) < -127) ? -127 : (turn + power);
    int left_side = ((turn - power) < -127) ? -127 : (turn - power);
  }
  else
  {
    int right_side = power;
    int left_side = -power;
  }
  FL = left_side;
  ML = -left_side;
  BL = left_side;
  FR = right_side;
  MR = -right_side;
  BR = right_side;
}


void FifteenInch::drive(void *ptr){
  while(true){
    int power = master.get_analog(ANALOG_RIGHT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    int four_bar_power = master.get_analog(ANALOG_LEFT_Y);

    tank_drive(power, turn);
    four_bar = four_bar_power;
    delay(5);
  }
}



void FifteenInch::move_to(void *ptr){

}
