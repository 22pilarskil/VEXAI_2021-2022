
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

std::atomic<double> FifteenInch::x = 0;

Motor FifteenInch::FL(17, true);
Motor FifteenInch::ML(18, true);
Motor FifteenInch::BL(19, true);
Motor FifteenInch::FR(1);
Motor FifteenInch::MR(2);
Motor FifteenInch::BR(3);
Motor FifteenInch::four_bar(20, true);

Imu FifteenInch::IMU(15);
Rotation FifteenInch::left_dead_wheel(21);
Rotation FifteenInch::right_dead_wheel(14);

std::map<std::string, std::unique_ptr<pros::Task>> FifteenInch::tasks;
void FifteenInch::tank_drive(int power, int turn)
  {
    int left_side;
    int right_side;


    left_side = power + turn;
    right_side = power - turn;


    FL = -left_side;
    ML = left_side;
    BL = -left_side;
    FR = right_side;
    MR = -right_side;
    BR = right_side;
  }


  void FifteenInch::drive(void *ptr){
    while(true){
      int power = master.get_analog(ANALOG_LEFT_Y);
      int turn = master.get_analog(ANALOG_RIGHT_X);
      int four_bar_power = master.get_analog(ANALOG_LEFT_Y);

      tank_drive(power, turn);
      four_bar = four_bar_power;
      delay(5);
    }
  }

  void FifteenInch::start_task(std::string name, void (*func)(void *)) {
      if (!task_exists(name)) {
          tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
      }
  }

  bool FifteenInch::task_exists(std::string name) {
      return tasks.find(name) != tasks.end();
  }

  void FifteenInch::kill_task(std::string name) {
      if (task_exists(name)) {
          tasks.erase(name);
      }
  }


  void FifteenInch::move_to(void *ptr){

  }
