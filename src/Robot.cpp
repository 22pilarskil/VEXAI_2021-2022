#include "main.h"
#include "Robot.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "PD.h"
// #include "PD.cpp"
#include <map>
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
using namespace pros;
using namespace std;


std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;
std::unordered_map<std::string, Motor> Robot::motor_map;

Controller Robot::master(E_CONTROLLER_MASTER);
PD Robot::power_PD(.32, 5, 0);
PD Robot::strafe_PD(.17, .3, 0);
PD Robot::turn_PD(10, 1, 0);

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::new_x = 0;
std::atomic<double> Robot::new_y = 0;
std::atomic<double> Robot::heading = 0;
std::atomic<double> Robot::imu_val = 0;
std::atomic<bool> Robot::chasing_mogo = false;



//24 inch declarations
double Robot::offset_back = 5.25;
double Robot::offset_middle = 7.625;
Motor Robot::BLT(1); //front left top
Motor Robot::BLB(3, true); //front left bottom
Motor Robot::BRT(10, true); //front right top
Motor Robot::BRB(9); //front right bottom
Motor Robot::FRT(19, true); //back right top
Motor Robot::FRB(18); //back right bottom
Motor Robot::FLT(13); //back left top
Motor Robot::FLB(11, true); //back left bottom
Motor Robot::angler(20);
Motor Robot::conveyor(2);
Motor Robot::flicker(17);
ADIEncoder Robot::LE({{16, 5, 6}});
ADIEncoder Robot::RE({{16, 1, 2}});
ADIEncoder Robot::BE({{16, 3, 4}});
ADIAnalogIn Robot::potentiometer({{16, 8}});
ADIDigitalOut Robot::piston(1);
Gps Robot::gps(5, 1.2192, -1.2192, 180, 0, .4064);
Imu Robot::IMU(12);
Distance Robot::angler_dist(21);
Distance Robot::mogo_dist(15);


//test bot declarations
/*
Imu Robot::IMU(15);
ADIEncoder Robot::LE(5, 6);
ADIEncoder Robot::RE(3, 4);
ADIEncoder Robot::BE(7, 8);
Distance Robot::mogo_dist(9);
Motor Robot::FR(17, true);
Motor Robot::FL(8);
Motor Robot::BR(3, true);
Motor Robot::BL(10);
double Robot::offset_back = 2.875;
double Robot::offset_middle = 5.0;
*/



double pi = 3.141592653589793238;
double Robot::wheel_circumference = 2.75 * pi;

const double inches_to_encoder = 41.669;
const double meters_to_inches = 39.3701;

// void Robot::add_motor(void *ptr)
// {
//   // Currently declarations for test bot
//   motor_map.insert({"front_right", Robot::FR});
//   motor_map.insert({"front_left", Robot::FL});
//   motor_map.insert({"back_right", Robot::BR});
//   motor_map.insert({"back_left", Robot::BL});

// }

void Robot::imu_clamp(void *ptr){
    double offset = 0;
    while (true){
        double rotation = IMU.get_rotation();
        if (abs(imu_val - rotation) < 10) offset = 0;
        else if (rotation > imu_val) offset = -360;
        else if (rotation < imu_val) offset = 360;

        imu_val = rotation + offset;
        delay(5);
    }
}

void Robot::receive_mogo(nlohmann::json msg) {

    double angle_threshold = 10;
    double lookahead_distance = 4;

    if (!task_exists("DEPTH")) start_task("DEPTH", Robot::check_depth);

    string msgS = msg.dump();
    std::size_t found = msgS.find(",");

    double lidar_depth = std::stod(msgS.substr(1, found - 1));
    double angle = std::stod(msgS.substr(found + 1, msgS.size() - found - 1));

    if (angle != 0) chasing_mogo = true;

    heading = (IMU.get_rotation() + angle);

    if (abs(angle) < angle_threshold && angle != 0){
        lidar_depth = (lidar_depth * meters_to_inches) * inches_to_encoder;
        new_y = y + lidar_depth * cos(heading / 180 * pi) * lookahead_distance;
        new_x = x + lidar_depth * sin(heading / 180 * pi) * lookahead_distance;

        lcd::print(5, "X: %f Y: %f L: %f", (float)new_y, (float)new_x, (float)lidar_depth);
        lcd::print(6, "Heading: %f Angle: %f", (float)heading, (float)angle);
    }
    else if (angle == 0 && chasing_mogo == true){
        new_y = new_y + 200;
    }
}

void Robot::check_depth(void *ptr){

    double depth_threshold = 10;
    std::deque<double> depth_vals;

    while(true){

        if ((int)depth_vals.size() == 10) depth_vals.pop_front();
        depth_vals.push_back(mogo_dist.get());
        double sum = 0;
        for (int i = 0; i < depth_vals.size(); i++) sum += depth_vals[i];
        double depth_average = sum / 10;

        if (abs(depth_average - mogo_dist.get()) < 1 && mogo_dist.get() > 0 && mogo_dist.get() < 50){
            new_x = (float)x;
            new_y = (float)y;
            lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#stop#");

            chasing_mogo = false;
            piston.set_value(false);
            delay(1000);
            start_task("ANGLER", Robot::depth_angler);

            kill_task("DEPTH");
        }
        delay(5);
    }
}

void Robot::depth_angler(void *ptr){
    int potentiometer_threshold = 270;
    int depth_threshold = 42;
    int depth_coefficient = 10;
    while (true){

        if(master.get_digital(DIGITAL_Y)) break;

        if(master.get_digital(DIGITAL_DOWN)) {
            while (potentiometer.get_value() < 2415){
                angler = -127;
            }
            break;
        }

        if (abs(potentiometer.get_value() - potentiometer_threshold) > 50){
            if (potentiometer.get_value() > potentiometer_threshold) angler = 127;
            else angler = -127;
        }
        else {
            double distance = angler_dist.get();
            if (abs(distance - depth_threshold) < 10) angler = depth_coefficient * (angler_dist.get() - depth_threshold);
            else angler = 0;
        }
        delay(5);
    }
    kill_task("ANGLER");
}

void Robot::drive(void *ptr) {
    while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool angler_forward = master.get_digital(DIGITAL_L1);
        bool angler_backward = master.get_digital(DIGITAL_L2);

        bool  angler_start_thread = master.get_digital(DIGITAL_X);

        bool piston_open = master.get_digital(DIGITAL_A);
        bool piston_close = master.get_digital(DIGITAL_B);

        bool conveyor_forward = master.get_digital(DIGITAL_R1);
        bool conveyor_backward = master.get_digital(DIGITAL_R2);

        bool flicker_on = master.get_digital(DIGITAL_UP);


        if (angler_backward) angler = 40;
        else if (angler_forward) angler = -40;
        else angler = 0;

        if (angler_start_thread && !task_exists("ANGLER")) start_task("ANGLER", Robot::depth_angler);

        if (piston_open) piston.set_value(true);
        else if (piston_close) piston.set_value(false);

        if (conveyor_forward) conveyor = 100;
        else if (conveyor_backward) conveyor = -100;
        else conveyor = 0;

        if (flicker_on) flicker = 127;
        else flicker = 0;

        mecanum(power, strafe, turn);
        delay(5);
    }
}

void Robot::mecanum(int power, int strafe, int turn, int max_power) {
    // max_power = 127
    int powers[] {
        power + strafe + turn,
        power - strafe - turn,
        power - strafe + turn,
        power + strafe - turn
    };

    int max = *max_element(powers, powers + 4);
    int min = abs(*min_element(powers, powers + 4));

    double true_max = double(std::max(max, min));
    double scalar = (true_max > max_power) ? max_power / true_max : 1;

    // FL = (power + strafe + turn) * scalar;
    // FR = (power - strafe - turn) * scalar;
    // BL = (power - strafe + turn) * scalar;
    // BR = (power + strafe - turn) * scalar;

    FLT = (power + strafe + turn) * scalar;
    FRT = (power - strafe - turn) * scalar;
    BLT = (power - strafe + turn) * scalar;
    BRT = (power + strafe - turn) * scalar;
    FLB = (power + strafe + turn) * scalar;
    FRB = (power - strafe - turn) * scalar;
    BLB = (power - strafe + turn) * scalar;
    BRB = (power + strafe - turn) * scalar;
}


void Robot::start_task(std::string name, void (*func)(void *)) {
    if (!task_exists(name)) {
        tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
    }
}

bool Robot::task_exists(std::string name) {
    return tasks.find(name) != tasks.end();
}

void Robot::kill_task(std::string name) {
    if (task_exists(name)) {
        tasks.erase(name);
    }
}

void Robot::fps(void *ptr) {
    double last_x = 0;
    double last_y = 0;
    double last_phi = 0;
    double turn_offset_y = 0;
    double turn_offset_x = 0;

    while (true) {
        double cur_phi = IMU.get_rotation() * pi / 180;
        double dphi = cur_phi - last_phi;

        double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
        double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;

        turn_offset_x = turn_offset_x + cur_turn_offset_x;
        turn_offset_y = turn_offset_y + cur_turn_offset_y;

        double cur_y = ((LE.get_value() - turn_offset_y) - (RE.get_value() + turn_offset_y)) / -2;
        double cur_x = BE.get_value() - turn_offset_x;

        double dy = cur_y - last_y;
        double dx = cur_x - last_x;

        double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
        double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);

        y = (float)y + global_dy;
        x = (float)x + global_dx;

        lcd::print(1,"Y: %f - X: %f", (float)y, (float)x, IMU.get_rotation());
        lcd::print(2, "IMU value: %f", IMU.get_heading());
        lcd::print(3, "TASK EXISTS %d", task_exists("DEPTH"));
        lcd::print(4,"Potentiometer %d - Dist. %d", potentiometer.get_value(), angler_dist.get());

        last_y = cur_y;
        last_x = cur_x;
        last_phi = cur_phi;

        delay(5);
    }
}

void Robot::gps_fps(void *ptr){
    while (true){
        lcd::print(1, "Y: %f - X: %f", (float)gps.get_status().y, (float)gps.get_status().x);
        lcd::print(2, "Heading: %f", (float)gps.get_heading());
        delay(5);
    }
}


void Robot::brake(std::string mode)
{

    if (mode.compare("coast") == 0)
    {
        FLT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        FLB.set_brake_mode(E_MOTOR_BRAKE_COAST);
        FRT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        FRB.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BLT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BLB.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BRT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BRB.set_brake_mode(E_MOTOR_BRAKE_COAST);

    }
    else if (mode.compare("hold") == 0)
    {
        FLT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FLB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FRT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FRB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BLT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BLB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BRT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BRB.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    }
    else FLT = FLB= FRT = FRB= BLT = BLB = BRT = BRB= 0;
}
void Robot::move_to(void *ptr)
{
    while (true)
    {

        double imu_error = imu_val - heading;
        double sign = (imu_error > 0) ? 1 : -1;
        imu_error = imu_error * imu_error * imu_error;
        double y_error = new_y - y;
        double x_error = new_x - x;

        double phi = (IMU.get_rotation()) * pi / 180;
        double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
        double turn = turn_PD.get_value(imu_error);
        turn = (abs(turn) < 15) ? turn : abs(turn)/turn * 15;

        mecanum(power, strafe, turn, 50);

        delay(5);
    }
}
