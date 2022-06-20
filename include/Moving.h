#ifndef _Moving
#define _Moving
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include "system/json.hpp"
#include "PID.h"

#include <atomic>
using namespace pros;
class Moving {
  public:
    static Motor FL;
    static Motor ML;
    static Motor BL;
    static Motor FR;
    static Motor MR;
    static Motor BR;

    static Rotation deadwheel;
    static Motor four_bar;
    static std::atomic<double> deadwheel_offset;
    static double cur_x_gps;
    static double cur_y_gps;
    static double cur_heading_gps;
    static Gps gps;
    static PID angle_pid;
    static PID distance_pid;
    static void gps_init(void *ptr);
    static Imu IMU;
    static void tank_drive(int power, int turn);
    static void move_to_gps(void *ptr);
    static void move_to_fps(void *ptr);
    static void move_to_set(double x, double y);
    static void update_fps(void *ptr);
    static bool running;
    static double move_to_x;
    static double move_to_y;
    static std::atomic<double> fps_x;
    static std::atomic<double> fps_y;
    static void toggleRunning();
    static void turn_to(double x);
    static void turn_to_fps(double x, double y);
    static void forward_by(double x);
    static void drive(void *ptr);
    static void start_task(std::string name, void (*func)(void *));
    static bool task_exists(std::string name);
		static void kill_task(std::string name);
    static double angle_difference(double x, double y);
    static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

};
#endif
