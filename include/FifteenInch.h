#ifndef _FifteenInch
#define _FifteenInch
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include "system/json.hpp"
using namespace pros;
class FifteenInch {
  public:
    static std::atomic<double> x;
    static Controller controller;
    static Controller master;
    static std::map<std::string, std::unique_ptr<pros::Task>> tasks;
    static Motor FL;
    static Motor ML;
    static Motor BL;
    static Motor FR;
    static Motor MR;
    static Motor BR;
    static Motor four_bar;
    static double cur_x_gps;
    static double cur_y_gps;
    static double cur_heading_gps;
    static Gps gps;


    static Imu IMU;
    static Rotation left_dead_wheel;
    static Rotation right_dead_wheel;
    // can't find radio api, will code for it once found

    static void drive(void *ptr);
    static void receive_data(nlohmann::json msg);
    static void send_data();
    static void tank_drive(int power, int turn);
    static void move_to(void *ptr);
    static void start_task(std::string name, void (*func)(void *));
    static void gps_initialize(void *ptr);
    static void gps_test(void *ptr);
    static bool task_exists(std::string name);
    static void kill_task(std::string name);
};
#endif
