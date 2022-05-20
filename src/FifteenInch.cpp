
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
#include <utility> //for pairs
using namespace pros;
using namespace std;
const double inches_to_encoder = 41.669;
const double meters_to_inches = 39.3701;
const double pi = 3.141592653589793238;




Controller FifteenInch::master(E_CONTROLLER_MASTER);

std::atomic<double> FifteenInch::x = 0;

Motor FifteenInch::FL(15, true);
Motor FifteenInch::ML(14, true);
Motor FifteenInch::BL(16, true);
Motor FifteenInch::FR(12);
Motor FifteenInch::MR(13);
Motor FifteenInch::BR(11);
Motor FifteenInch::four_bar(20, true);
ADIDigitalOut FifteenInch::arm_pistons(1); //port value not accurate

//variables relating to bot state. maybe combine into a struct at some point.
std::atomic<double> FifteenInch::cur_x_gps;
std::atomic<double> FifteenInch::cur_y_gps;
std::atomic<double> FifteenInch::cur_heading_gps;
std::atomic<bool> arms_down = false;

Gps FifteenInch::gps(1);

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
  FR = -right_side;
  MR = -right_side;
  BR = -right_side;
}
 // te

void FifteenInch::drive(void *ptr){
  while(true){
    int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_RIGHT_X);
    int four_bar_power = master.get_analog(ANALOG_LEFT_Y);
    //lcd::print(7, "%f, %f, %f", (float)cur_x_gps, (float)cur_y_gps, (float)cur_heading_gps);
    tank_drive(power, turn);
    four_bar = four_bar_power;
    delay(5);
  }
}
void FifteenInch::gps_initialize(void *ptr)
{
    double degree_offset = 0; // gps offset from forward; counterclockwise is positive
    double x_offset = (double)5/meters_to_inches; // x offset on bot of gps from the bot's frame of reference
    double y_offset = (double)0.5/meters_to_inches; // same as x but for y
    double new_x_offset = 0;
    double new_y_offset = 0;
    while (true){

      pros::c::gps_status_s cur_status = gps.get_status();
      cur_heading_gps = 360.0 - (double)gps.get_heading() + 90 + degree_offset;
      if (cur_heading_gps < 0) cur_heading_gps = cur_heading_gps + (std::atomic<double>)360.0;
      if (cur_heading_gps >= 360) cur_heading_gps = cur_heading_gps - (std::atomic<double>)360.0;
      //uncomment these lines when real offsets are put in place. Leave them commented if x_offset is 0 to avoid / by 0
      new_x_offset = sqrt(pow(x_offset, 2) + pow(y_offset, 2)) * cos(atan(y_offset / x_offset) + cur_heading_gps * 3.14159 / 180); //totally not burke's trig
      new_y_offset = sqrt(pow(x_offset, 2) + pow(y_offset, 2)) * sin(atan(y_offset / x_offset) + cur_heading_gps * 3.14159 / 180);
      cur_x_gps = (double)cur_status.x + new_x_offset;
      cur_y_gps = (double)cur_status.y + new_y_offset;
      //lcd::print(6, "%f, %f, %f", cur_x_gps, cur_y_gps, cur_heading_gps);
      delay(20);
  }
}
void FifteenInch::send_data() {
    std::string return_string = "#";
    for (int i = 1; i < 37; i++) {

        int ring_count = gridMapper->getBox(i)["ring"];
        int mogo_count = gridMapper->getBox(i)["mogo"];
        if(ring_count > 0 || mogo_count > 0)
        {
          return_string += std::to_string(i) + "#y#" + std::to_string(ring_count) + " " + std::to_string(mogo_count) + "#@#";
        }

    }
    //"#R#" + std::to_string(cur_x_gps) + " " + std::to_string(cur_y_gps) + " " + std::to_string(cur_heading_gps) + "#" + 
    double unit_circle = 90-cur_heading_gps;
    if(unit_circle <0){
      unit_circle += 360;
    }
    return_string = "#x#" + to_string(cur_x_gps) + "#y#"+to_string(cur_y_gps)+"#z#"+to_string(unit_circle)+"#";
    return_string = return_string +"@#";
    lcd::print(7,"%s",return_string);

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
  //double position_temp[] = {cur_y_gps * meters_to_inches / 24.0 + 3, cur_x_gps * meters_to_inches / 24.0 + 3, cur_heading_gps * 3.14159 / 180}; // I'm assuming this to be the point near the corner we tested from the first time
  //lcd::print(7, "%f, %f, %f", (float)position_temp[0], (float)position_temp[1], (float)position_temp[2]);
  double position_temp[] = {1.0, 1.0, 3.1415/4}; // I'm assuming this to be the point near the corner we tested from the first time
  //lcd::print(7, "%f, %f, %f", (float)position_temp[0], (float)position_temp[1], (float)position_temp[2]);

  gridMapper->map(position_temp, objects);

  for (int i = 0; i < 5; i++) { // TEST CODE
       std::string print_string = "";
       for (int j = 1; j <= 6; j++) {

            print_string += std::to_string(j + 6 * i) + ":" + std::to_string(gridMapper->getBox((j + 6 * i))["mogo"]) + " ";

       }
       lcd::print((i+1), print_string.c_str());
       delay(5);
  }



  send_data();

}

void FifteenInch::arm_change() {
    arm_pistons.set_value(!arms_down);
    arms_down = !arms_down;
}

//function is NOT complete
//any comment with + at the beginning is waiting on david finishing move_to
void FifteenInch::autonomous(void *ptr) {
    //setting initial conditions
    if (arms_down) {
        FifteenInch::arm_change();
    }



    //all x and y variables are in meters and heading values are assumed to be based on unit circle degrees
    //all variables that should be ajusted/changed/tuned are here at the beginning

    //these 2 relate to starting pos on the field
    bool north = false;
    bool blue_team = false;

    //all of these are measured 
    double mogo_x = 0;
    double north_mogo_y = 0;
    double mid_mogo_y = 0;
    double south_mogo_y = 0;

    double bot_mogo_dist = 0; //this is the distance the bot should be from the mogos when both arms are in the buckets. Measured from center of bot to center of mogos (x val only)
    double backup_dist = 0; //distance we back up after grabbing both mogos with the arms
    double grab_dist = 0; //distance away from the center of a mogo we should be for a clamp. Measured from center of bot
    double arm_delay = 0; //time between telling arms to move and them being in position. Can be solved by a bool value being returned


    //construction for all mogo positions. current positions are temporary
    pair<double, double> north_mogo_pos;
    north_mogo_pos.first = mogo_x;
    north_mogo_pos.second = north_mogo_y;
    pair<double, double> mid_mogo_pos;
    mid_mogo_pos.first = mogo_x;
    mid_mogo_pos.second = mid_mogo_y;
    pair<double, double> south_mogo_pos;
    south_mogo_pos.first = mogo_x;
    south_mogo_pos.second = south_mogo_y;

    pair<double, double> grab_2_pos; //position that the bot should go to to be in the right spot to grab both mogos

    //these pair values are not correct until after the if(north == blue_team) statement
    //also all the assignments are very confusing in random if statements but that allows for reusing of if statements
    pair<double, double> left_mogo_pos;
    pair<double, double> right_mogo_pos;

    if (north) {
        grab_2_pos.second = (north_mogo_pos.second + mid_mogo_pos.second) / 2;
    } else {
        grab_2_pos.second = (south_mogo_pos.second + mid_mogo_pos.second) / 2;
    }

    double goto_heading = 0;
    if (blue_team) {
        grab_2_pos.first = mogo_x + bot_mogo_dist;
        goto_heading = 180;
        left_mogo_pos.first = bot_mogo_dist;
        right_mogo_pos.first = bot_mogo_dist;
        left_mogo_pos.second = south_mogo_y;
        right_mogo_pos.second = north_mogo_y;
    } else {
        grab_2_pos.first = mogo_x - bot_mogo_dist;
        left_mogo_pos.first = -bot_mogo_dist;
        right_mogo_pos.first = -bot_mogo_dist;
        left_mogo_pos.second = north_mogo_y;
        right_mogo_pos.second = south_mogo_y;
    }

    //these two move_tos grab the mogos with the arms then pull them back
    //+while (move_to(grab_2_pos.first, grab_2_pos.second, goto_heading)) delay(5);
    FifteenInch::arm_change();
    delay(arm_delay); //tune this delay to allow time for arms to get in place, or start arms early, or have a return true from arms when they finish lowering
    //+while (move_to(grab_2_pos.first - backup_dist, grab_2_pos.second, goto_heading)) delay(5);

    FifteenInch::arm_change();
    delay(arm_delay); //same comment as other delay; probably use a shorter delay

    if (north == blue_team) {
        left_mogo_pos.first += mogo_x;
        left_mogo_pos.second = mid_mogo_y;
        right_mogo_pos.first += mogo_x;
    } else {
        left_mogo_pos.first += mogo_x;
        right_mogo_pos.first += mogo_x;
        right_mogo_pos.second = mid_mogo_y;
    }

    //+while(move_to(left_mogo_pos.first, left_mogo_pos.second, atan((left_mogo_pos.second - cur_y_gps)/(left_mogo_pos.first - cur_x_gps)))) delay(5);
    //.clamp() (integrate depth sensors into when we want to do this)
    //+while(move_to(right_mogo_pos.first, right_mogo_pos.second, atan((right_mogo_pos.second - cur_y_gps)/(right_mogo_pos.first - cur_x_gps)))) delay(5);
    //.clamp()

    //end conditions: bot holds middle yellow mogo and one side mogo

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
