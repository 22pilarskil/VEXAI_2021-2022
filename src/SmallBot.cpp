
#include "main.h"
#include "SmallBot.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "system/Data.h"
#include "utils/PD.h"
#include "utils/GridMapper.cpp"
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




Controller SmallBot::master(E_CONTROLLER_MASTER);

std::atomic<double> SmallBot::x = 0;

Motor SmallBot::FL(1, true);
Motor SmallBot::ML(3, true);
Motor SmallBot::BL(5, true);
Motor SmallBot::FR(4);
Motor SmallBot::MR(2);
Motor SmallBot::BR(6);
Motor SmallBot::four_bar(20, true);
double SmallBot::cur_x_gps;
double SmallBot::cur_y_gps;
double SmallBot::cur_heading_gps;
Gps SmallBot::gps(16);

int s = 0;
Imu SmallBot::IMU(15);
Rotation SmallBot::BE(20);
GridMapper* smallBotGrid = new GridMapper();
std::map<std::string, std::unique_ptr<pros::Task>> SmallBot::tasks;
void SmallBot::tank_drive(int power, int turn)
{
  int left_side;
  int right_side;

  left_side = power + turn;
  right_side = power - turn;


  FL = -left_side;
  ML = left_side;
  BL = -left_side;
  FR = -right_side;
  MR = right_side;
  BR = -right_side;
}
 // te

void SmallBot::drive(void *ptr){
  while(true){
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    int four_bar_power = master.get_analog(ANALOG_LEFT_Y);
    lcd::print(7, "%f, %f, %f", (float)cur_x_gps, (float)cur_y_gps, (float)cur_heading_gps);
    tank_drive(power, turn);
    four_bar = four_bar_power;
    delay(5);
  }
}
void SmallBot::gps_initialize(void *ptr)
{
    double degree_offset = 0; // gps offset from forward; counterclockwise is positive
    double x_offset = (double)5/meters_to_inches; // x offset on bot of gps from the bot's frame of reference
    double y_offset = (double)0.5/meters_to_inches; // same as x but for y
    double new_x_offset = 0;
    double new_y_offset = 0;
    while (true){

      pros::c::gps_status_s cur_status = gps.get_status();
      cur_heading_gps = 360.0 - (double)gps.get_heading() + 90 + degree_offset;
      if (cur_heading_gps < 0) cur_heading_gps += 360;
      if (cur_heading_gps >= 360) cur_heading_gps -= 360;
      //uncomment these lines when real offsets are put in place. Leave them commented if x_offset is 0 to avoid / by 0
      new_x_offset = sqrt(pow(x_offset, 2) + pow(y_offset, 2)) * cos(atan(y_offset / x_offset) + cur_heading_gps * 3.14159 / 180); //totally not burke's trig
      new_y_offset = sqrt(pow(x_offset, 2) + pow(y_offset, 2)) * sin(atan(y_offset / x_offset) + cur_heading_gps * 3.14159 / 180);
      cur_x_gps = (double)cur_status.x + new_x_offset;
      cur_y_gps = (double)cur_status.y + new_y_offset;
      //lcd::print(6, "%f, %f, %f", cur_x_gps, cur_y_gps, cur_heading_gps);
      delay(20);
  }
}
void SmallBot::send_data() {
    std::string return_string = "#";
    for (int i = 1; i < 37; i++) {

        int ring_count = smallBotGrid->getBox(i)["ring"];
        int mogo_count = smallBotGrid->getBox(i)["mogo"];
        if(ring_count > 0 || mogo_count > 0)
        {
          return_string += std::to_string(i) + "#" + std::to_string(ring_count) + " " + std::to_string(mogo_count) + "#@#";
        }

    }
    //"#R#" + std::to_string(cur_x_gps) + " " + std::to_string(cur_y_gps) + " " + std::to_string(cur_heading_gps) + "#" + 
    double unit_circle = 90-cur_heading_gps;
    if(unit_circle <0){
      unit_circle += 360;
    }
    return_string = "#x#" + to_string(cur_x_gps) + "#y#"+to_string(cur_y_gps)+"#z#"+to_string(unit_circle)+"#";
    return_string = return_string +"@#";
    //lcd::print(7,"%s",return_string);

    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, return_string);





}

void SmallBot::receive_data(nlohmann::json msg){
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
  //double position_temp[] = {cur_y_gps * meters_to_inches / 24.0 + 3, cur_x_gps * meters_to_inches / 24.0 + 3, cur_heading_gps * 3.14159 / 180}; // I'm assuming this to be the point near the corner we tested from the first time
  //lcd::print(7, "%f, %f, %f", (float)position_temp[0], (float)position_temp[1], (float)position_temp[2]);
  double position_temp[] = {1.0, 1.0, 3.1415/4}; // I'm assuming this to be the point near the corner we tested from the first time
  //lcd::print(7, "%f, %f, %f", (float)position_temp[0], (float)position_temp[1], (float)position_temp[2]);

  smallBotGrid->map(position_temp, objects);

  for (int i = 0; i < 5; i++) { // TEST CODE
       std::string print_string = "";
       for (int j = 1; j <= 6; j++) {

            print_string += std::to_string(j + 6 * i) + ":" + std::to_string(smallBotGrid->getBox((j + 6 * i))["mogo"]) + " ";

       }
       lcd::print((i+1), print_string.c_str());
       delay(5);
  }



  send_data();

}
void SmallBot::gps_test(void *ptr)
{
  while(true)
  {
    //lcd::print(0, "%f, %f, %f", (float)cur_x_gps, (float)cur_y_gps, (float)(360-cur_heading_gps));
    delay(5);
  }
}
void SmallBot::start_task(std::string name, void (*func)(void *)) {
    if (!task_exists(name)) {
        tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
    }
}

bool SmallBot::task_exists(std::string name) {
    return tasks.find(name) != tasks.end();
}

void SmallBot::kill_task(std::string name) {
    if (task_exists(name)) {
        tasks.erase(name);
    }
}


void SmallBot::move_to(void *ptr){

}
