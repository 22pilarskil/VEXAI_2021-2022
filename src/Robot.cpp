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

Controller Robot::master(E_CONTROLLER_MASTER);
// Motor Robot::FLT(2, true); //front left top
// Motor Robot::FLB(1); //front left bottom
// Motor Robot::FRT(9); //front right top
// Motor Robot::FRB(10, true); //front right bottom
// Motor Robot::BRT(20, true); //back right top
// Motor Robot::BRB(19); //back right bottom
// Motor Robot::BLT(11); //back left top
// Motor Robot::BLB(12, true); //back left bottom
// Motor Robot::roller(18); //mechanism for ascending rings
Motor Robot::FR(17, true);
Motor Robot::FL(8);
Motor Robot::BR(3, true);
Motor Robot::BL(10);
Motor Robot::angler(17);
Motor Robot::conveyor(18);

Imu Robot::IMU(15);
ADIEncoder Robot::LE(5, 6);
ADIEncoder Robot::RE(3, 4);
ADIEncoder Robot::BE(7, 8);
ADIDigitalOut Robot::piston(2);
ADIAnalogIn Robot::potentiometer(1);
Distance Robot::dist(9);

PD Robot::power_PD(.32, 5, 0);
PD Robot::strafe_PD(.17, .3, 0);
PD Robot::turn_PD(2.4, 1, 0);

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::new_x = 0;
std::atomic<double> Robot::new_y = 0;
std::atomic<double> Robot::heading = 0;

double Robot::offset_back = 2.875;
double Robot::offset_middle = 5.0;
double pi = 3.141592653589793238;
double Robot::wheel_circumference = 2.75 * pi;

double angle_threshold = 5;
double depth_threshold1 = 200;
double depth_threshold2 = 50;
double depth_coefficient1 = .2;
double depth_coefficient2 = .02;


void Robot::receive(nlohmann::json msg) {

	double depth_coefficient = depth_coefficient1;
    string msgS = msg.dump();
    std::size_t found = msgS.find(",");

    double lidar_depth = std::stod(msgS.substr(1, found - 1));
    double angle = std::stod(msgS.substr(found + 1, msgS.size() - found - 1));
    double phi = IMU.get_rotation() * pi / 180; //should this be IMU.get_rotation() or heading?
    double depth;

    heading = (IMU.get_rotation() - angle);
    bool movement_over = false;

    if (abs(angle) < angle_threshold){
    	do {
    		depth = dist.get();
	        double change = depth * depth_coefficient;
	        new_y = (float)new_y + change * cos(phi);
	        new_x = (float)new_x - change * sin(phi);
	        if (depth < depth_threshold1 && !movement_over){
	        	lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#stop#");
	        	movement_over = true;
	        }
	        delay(5);
	        depth_coefficient = depth_coefficient2;
	    } while (depth < depth_threshold1 && depth > depth_threshold2);
    }

    if (movement_over){
    	new_y = (float)y;
    	new_x = (float)x;
    	lcd::print(7, "MOGO REACHED");
    }
    lcd::print(5, "X: %f Y: %f", (float)new_x, (float)new_y);
    lcd::print(6, "Heading: %f Angle: %f", (float)heading, (float)angle);
}

void Robot::reset_PD() {
    power_PD.reset();
    strafe_PD.reset();
    turn_PD.reset();
}

void Robot::drive(void *ptr) {
    while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool angler_forward = master.get_digital(DIGITAL_L1);
        bool angler_backward = master.get_digital(DIGITAL_L2);

        bool piston_open = master.get_digital(DIGITAL_A);
        bool piston_close = master.get_digital(DIGITAL_B);

        bool conveyor_forward = master.get_digital(DIGITAL_R1);
        bool conveyor_backward = master.get_digital(DIGITAL_R2);

        if (angler_forward) angler = 50;
        else if (angler_backward) angler = -50;
        else angler = 0;

        if (piston_open) piston.set_value(true);
        else if (piston_close) piston.set_value(false);
        
        if (conveyor_forward) conveyor = 100;
        else if (conveyor_backward) conveyor = -100;
        else conveyor = 0;
        
        //mecanum(power, strafe, turn);
        delay(5);
    }
}

// void Robot::mecanum(int power, int strafe, int turn) {

//  int powers[] {
//      power + strafe + turn,
//      power - strafe - turn,
//      power - strafe + turn, 
//      power + strafe - turn
//  };

//  int max = *max_element(powers, powers + 4);
//  int min = abs(*min_element(powers, powers + 4));

//  double true_max = double(std::max(max, min));
//  double scalar = (true_max > 127) ? 127 / true_max : 1;
    
//  FLT = 0*(power + strafe + turn) * scalar;
//  FLB = 0*(power + strafe + turn) * scalar;

//  FRT = (power - strafe - turn) * scalar;
//  FRB = (power - strafe - turn) * scalar;
//  BLT = (power - strafe + turn) * scalar;
//  BLB = (power - strafe + turn) * scalar;
//  BRT = (power + strafe - turn) * scalar;
//  BRB = (power + strafe - turn) * scalar;
// }
void Robot::mecanum(int power, int strafe, int turn) {

    int powers[] {
        power + strafe + turn,
        power - strafe - turn,
        power - strafe + turn, 
        power + strafe - turn
    };

    int max = *max_element(powers, powers + 4);
    int min = abs(*min_element(powers, powers + 4));

    double true_max = double(std::max(max, min));
    double scalar = (true_max > 127) ? 127 / true_max : 1;
    scalar = 1; //try removing this line
    

    FL = (power + strafe + turn) * scalar;
    FR = (power - strafe - turn) * scalar;
    BL = (power - strafe + turn) * scalar;
    BR = (power + strafe - turn) * scalar;
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
        // lcd::print(2, "IMU value: %f", IMU.get_heading());
        // lcd::print(3, "LE: %d RE: %d", LE.get_value(), RE.get_value());
        // lcd::print(4, "BE: %d", BE.get_value());
        lcd::print(2, "Potentiometer: %d", potentiometer.get_value());

        last_y = cur_y;
        last_x = cur_x;
        last_phi = cur_phi;

        delay(5);
    }
}
void Robot::brake(std::string mode)
{

    if (mode.compare("coast") == 0)
    {
        //FLT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        //FLB.set_brake_mode(E_MOTOR_BRAKE_COAST);
        //FRT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        //FRB.set_brake_mode(E_MOTOR_BRAKE_COAST);
        //BLT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        //BLB.set_brake_mode(E_MOTOR_BRAKE_COAST);
        //BRT.set_brake_mode(E_MOTOR_BRAKE_COAST);
        //BRB.set_brake_mode(E_MOTOR_BRAKE_COAST);
        FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
        FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
        BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
    }
    else if (mode.compare("hold") == 0)
    {
        // FLT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        // FLB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        // FRT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        // FRB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        // BLT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        // BLB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        // BRT.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        // BRB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    }
    else FL = FR = BL = BR = 0;
}
void Robot::move_to(void *ptr) 
{
    while (true)
    { 

        double imu_error = -(IMU.get_rotation() - heading);
        double y_error = new_y - y;
        double x_error = -(new_x - x);

        double phi = (IMU.get_rotation()) * pi / 180;
        double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
        double turn = turn_PD.get_value(imu_error);
        turn = (abs(turn) < 15) ? turn : abs(turn)/turn * 15;

        mecanum(power, strafe, turn);

        delay(5);
    }
}



