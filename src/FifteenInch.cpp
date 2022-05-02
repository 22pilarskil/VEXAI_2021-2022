
#include "main.h"
#include "FifteenInch.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "system/Data.h"
#include "PD.h"
#include "GridMapper.cpp"
#include <map>
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
using namespace pros;
using namespace std;
const double inches_to_encoder = 41.669;
const double meters_to_inches = 39.3701;
const double pi = 3.141592653589793238;

Controller FifteenInch::master(E_CONTROLLER_MASTER);

std::atomic<double> FifteenInch::x = 0;

Motor FifteenInch::FL(17, true);
Motor FifteenInch::ML(18, true);
Motor FifteenInch::BL(19, true);
Motor FifteenInch::FR(1);
Motor FifteenInch::MR(2);
Motor FifteenInch::BR(3);
Motor FifteenInch::four_bar(20, true);
double FifteenInch::cur_x_gps;
double FifteenInch::cur_y_gps;
double FifteenInch::cur_heading_gps;
Gps FifteenInch::gps(5);

int s = 0;
Imu FifteenInch::IMU(15);
Rotation FifteenInch::left_dead_wheel(21);
Rotation FifteenInch::right_dead_wheel(14);
GridMapper* gridMapper = new GridMapper();
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
 // te

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
void FifteenInch::gps_initialize(void *ptr)
{
    double degree_offset = 0; // gps offset from forward; counterclockwise is positive
    double x_offset = 0; // x offset on bot of gps from the bot's frame of reference
    double y_offset = 0; // same as x but for y
    while (true){

      pros::c::gps_status_s cur_status = gps.get_status();
      cur_heading_gps = 360.0 - (double)gps.get_heading() - 90.0 - degree_offset;
      //uncomment these lines when real offsets are put in place. Leave them commented if x_offset is 0 to avoid / by 0
      //x_offset = sqrt(x_offset ** 2 + y_offset ** 2) * cos(atan(y_offset / x_offset) + cur_heading_gps * 3.14159 / 180); //totally not burke's trig
      //y_offset = sqrt(x_offset ** 2 + y_offset ** 2) * sin(atan(y_offset / x_offset) + cur_heading_gps * 3.14159 / 180);
      cur_x_gps = (double)cur_status.x + x_offset;
      cur_y_gps = (double)cur_status.y + y_offset;
      if (cur_heading_gps < 0) cur_heading_gps += 360;
      delay(20);
  }
}
void FifteenInch::send_data() {
    std::string return_string = "#";
    for (int i = 1; i < 37; i++) {
        return_string += std::to_string(i) + "#";
        int ring_count = gridMapper->getBox(i)["ring"];
        int mogo_count = gridMapper->getBox(i)["mogo"];
        return_string += std::to_string(ring_count) + " " + std::to_string(mogo_count) + "#";
    }
    return_string = return_string +"@#";
    // lcd::print(2,"%s",return_string);
    // lcd::print(4, "prepping send");

    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#@#");
    delay(250);//serial sometimes concatenates packets into 1, which makes the continue packet contain the actual data packet sometimes
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, return_string);





}

void FifteenInch::receive_data(nlohmann::json msg){
  s++;
  //double position_temp[] = {gps.get_status().x*meters_to_inches/24 + 3, gps.get_status().y*meters_to_inches/24 + 3, pi/4}; //angle is unit circle degrees with y+ at true north and x+ at true east
  //double position_temp[] = {0.0,0.0,pi/4};
  std::map<std::string, std::vector<std::vector<double>>> objects;
  string names[] = {"mogo", "ring"}; //had to switch to this b/c of the recent ID change to 0 = mogo and 1 = ring
  string x;
  // if (stop) return;
  //     started = true;
  std:vector<std::vector<float>> pred = Data::get_pred(msg);

  for (std::vector<float> det : pred) {
      std::vector<double> location = {(double)det[0] * meters_to_inches, (double)det[1]/180*pi};
      // location[0] = 1.6 * meters_to_inches;
      // location[1] = 28.0 / 180.0 * pi;
      objects[names[(int)det[2]]].push_back(location); //haven't touched any of this, so I'm assuming it to be right
  }

  //lcd::print(6, (std::to_string(objects["mogo"][0][1])).c_str());

  //this position_temp uses gps values. use if using gps
  //double position_temp[] = {cur_x_gps * meters_to_inches / 24.0 + 3, cur_y_gps * meters_to_inches / 24.0 + 3, cur_heading_gps * 3.14159 / 180}; // I'm assuming this to be the point near the corner we tested from the first time
  
  double position_temp[] = {1.0, 5.0, 3.14159/4}; // I'm assuming this to be the point near the corner we tested from the first time

  gridMapper->map(position_temp, objects);

  for (int i = 0; i < 5; i++) { // TEST CODE
       std::string print_string = "";
       for (int j = 1; j <= 6; j++) {
          print_string += std::to_string(j + 6 * i) + ":" + std::to_string(gridMapper->getBox((j + 6 * i))["mogo"]) + " ";
       }
       lcd::print((i+1), print_string.c_str());
       delay(5);
  }

  // send_data();

}
void FifteenInch::gps_test(void *ptr)
{
  while(true)
  {
    //lcd::print(0, "%f, %f, %f", (float)cur_x_gps, (float)cur_y_gps, (float)(360-cur_heading_gps));
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
