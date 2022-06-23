#include "main.h"
#include "Moving.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "SmallBot.h"
#include "system/Data.h"
#include "PID.h"
#include "utils/GridMapper.cpp"
#include <map>
#include <cmath>
#include <atomic>
#include <vector>
#include <math.h>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <atomic>
using namespace pros;
Motor Moving::FL(6, true);
Motor Moving::ML(2, true);
Motor Moving::BL(4, true);
Motor Moving::FR(5);
Motor Moving::MR(3);
Motor Moving::BR(1);
Rotation Moving::deadwheel(20);
//Motor Moving::four_bar(20, true);
PID Moving::angle_pid(1.65,0.01,0,2);
PID Moving::distance_pid(1.2,0.01,0,0.1);
Gps Moving::gps(16);


//int s =
Imu Moving::IMU(17);
double pi = 3.14159265358979323846;
std::atomic<double> Moving::deadwheel_offset = 4.125;
//double encoders_to_inches = 1;
double inches_to_meters = 1/39.37;
bool Moving::running = false;
double Moving::move_to_x;
double Moving::move_to_y;
std::atomic<double> Moving::fps_x;
std::atomic<double> Moving::fps_y;
std::map<std::string, std::unique_ptr<pros::Task>> Moving::tasks;

void Moving::start_task(std::string name, void (*func)(void *)) {

    if (!task_exists(name)) {
        tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &move_to_x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
    }
}

bool Moving::task_exists(std::string name) {
    return tasks.find(name) != tasks.end();
}

void Moving::kill_task(std::string name) {
    if (task_exists(name)) {
        tasks.erase(name);
    }
}

void Moving::tank_drive(int power, int turn)
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
  BR = right_side;
}
void Moving::move_to_set(double x, double y)
{
  move_to_x = x;
  move_to_y = y;
}
void Moving::toggleRunning()
{
  running = true;
}
void Moving::update_fps(void *ptr)
{
  deadwheel.set_position(0);
  double last_x = 0;
  double last_y = 0;
  double last_phi = 0;
  double turn_offset_x = 0;
  double turn_offset_y = 0;
  while (true) {
      double cur_phi = IMU.get_rotation() / 180 * pi;
      if(isnan(cur_phi) == 0 && isinf(cur_phi) == 0)
      {
        double dphi = cur_phi - last_phi;

        double cur_turn_offset_x = 360 * (deadwheel_offset * dphi) / (1.5*pi);

        turn_offset_x = (double)turn_offset_x + (double)cur_turn_offset_x;

        double cur_y = (double)(MR.get_position() + ML.get_position()) / 2;//average of 2 y's
        double cur_x = ((double)deadwheel.get_position()/100 + turn_offset_x);// over 100 b/c converting centidegrees to degrees

        double dy = cur_y - last_y;
        double dx = cur_x - last_x;

        double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
        double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);//deals with angles

        fps_y = fps_y - global_dy/(360)*3.25*pi*0.6;//multiply by circumference and gear ratio
        fps_x = fps_x + global_dx/(360)*1.5*pi;// just circumference b/c rotation sensor has no gear ratio

        last_y = cur_y;
        last_x = cur_x;
        last_phi = cur_phi;
        lcd::print(1, "%f, %f", (float)fps_x*inches_to_meters, (float)fps_y*inches_to_meters);
      }
      lcd::print(2, "%f", (float)cur_phi);
      delay(5);
  }

}
void Moving::move_to_fps(void *ptr)
{
  while(true)
  {
    double angle_thres = 5;
    double dist_thres = 0.15;//cm

    double distance_away = sqrt(pow((fps_x*inches_to_meters - move_to_x),2) + pow(fps_y*inches_to_meters-move_to_y, 2));
    lcd::print(3, "%f", distance_away);
    if(fps_x == 0 || fps_y == 0)
    {
      delay(5);
    }
    else if(distance_away < dist_thres)
    {
      tank_drive(0,0);
      break;
    }
    else
    {

      double h1 = atan2((-move_to_y + fps_y*inches_to_meters), (-move_to_x + fps_x*inches_to_meters));
      if(h1 < 0) h1 += 2*pi;
      double heading_to = 90 - 180/pi*h1;
      if(heading_to < 0) heading_to += 360;
      //upper segment transforms a unit_circle degree to a robot degree
      lcd::print(6,"%f",heading_to);
      if(abs(angle_difference(IMU.get_heading(),heading_to)) > (double)angle_thres)
      {
        //lcd::print(4, "%s", "TURNING");
        turn_to_fps(heading_to, 2);//does the turning
      }
      lcd::print(4, "%s", "NOW MOVING");
      tank_drive(50,0);//switch later w/ PID
      delay(5);
    }
  }
}
void Moving::turn_to_fps(double x, double error_thres)
{
  while(abs(angle_difference(IMU.get_heading(), x)) > (double)error_thres)
  {
    //double turn_by = angle_pid.get_value(change_in_heading - distance_turned);
    //lcd::print(1, "%d", angle_pid.get_value(50));
    double start = IMU.get_heading();
    int mult = (angle_difference(start, x)>0)?1:-1;//figure out the way to turn
    tank_drive(0,25*mult);
    //lcd::print(4, "%s", "turning");
    lcd::print(5, "%f, %f", (double)IMU.get_heading(), abs(angle_difference(IMU.get_heading(), x)));

    //lcd::print(3, "%f", abs(angle_difference(gps.get_heading(), turn_to)));
    delay(5);
  }
  tank_drive(0,0);
  delay(5);
}
void Moving::move_to_gps(void *ptr)
{
  //lcd::print(1, "%s", "hi");
  while(true)
  {
    double angle_thres = 5;
    double dist_thres = 15;//cm
    double cur_x_gps = gps.get_status().x;
    double cur_y_gps = gps.get_status().y;
    //if(running)
    //{
    delay(5000);
      double distance_away = sqrt(pow((cur_x_gps - move_to_x),2) + pow(cur_y_gps-move_to_y, 2)) * 100;
      lcd::print(1, "%f", distance_away);
      while(distance_away > dist_thres)
      {
        cur_x_gps = gps.get_status().x;
        cur_y_gps = gps.get_status().y;
        distance_away = sqrt(pow((cur_x_gps - move_to_x),2) + pow(cur_y_gps-move_to_y, 2)) * 100;

        double h1 = atan2((move_to_y-cur_y_gps), (move_to_x-cur_x_gps));
        if(h1 < 0) h1 += 2*pi;
        double heading_to = 90 - 180/pi*h1;
        if(heading_to < 0) heading_to += 360;

        //reflect it b/c gps is on other side of the front

        lcd::print(5, "%f", heading_to);
        if(abs(angle_difference(gps.get_heading(),heading_to)) > (double)angle_thres)
        {
          //lcd::print(4, "%s", "TURNING");
          turn_to(heading_to);
        }
        lcd::print(4, "%s", "NOW MOVING");
        tank_drive(-30,0);//switch later w/ PID
        delay(5);
      }
      tank_drive(0,0);
      break;
      //lcd::print(5, "%s","here");
    //}
    delay(5);
  }
}

double Moving::angle_difference(double start, double end)//get minimum distance between 2 angles including sign
{
  double end1 = end;
  double left = (start-end)>=0? start-end : start-end+360;
  double right = end-start>=0 ? end-start : end-start+360;;

  return (left<right)? -left : right;
}
void Moving::turn_to(double turn_to)//turn_to in degrees
{

  lcd::print(0, "Started");
  //cock dick balls penis
  double angle_thres = 2.5;//in degrees, take into account drift

  //lcd::print(1, "%f", abs(turn_amount - distance_turned));


  while(abs(angle_difference(gps.get_heading(), turn_to)) > (double)angle_thres)
  {
    //double turn_by = angle_pid.get_value(change_in_heading - distance_turned);
    //lcd::print(1, "%d", angle_pid.get_value(50));
    double start = gps.get_heading();
    int mult = (angle_difference(start, turn_to)>0)?1:-1;//figure out the way to turn
    tank_drive(0,40*mult);
    lcd::print(4, "%s", "turning");
    lcd::print(2, "%f, %f", (double)angle_thres, angle_difference(gps.get_heading(), turn_to));

    //lcd::print(3, "%f", abs(angle_difference(gps.get_heading(), turn_to)));
    delay(15);
  }
  tank_drive(0,0);
  delay(5);



  tank_drive(0,0);
//
	//tank_drive(0, 0); left and right side vel
}
