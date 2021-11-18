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



Imu Robot::IMU(15);
ADIEncoder Robot::LE(5, 6);
ADIEncoder Robot::RE(3, 4);
ADIEncoder Robot::BE(7, 8);
PD Robot::power_PD(.32, 5, 0);
PD Robot::strafe_PD(.17, .3, 0);
PD Robot::turn_PD(1.2, 1, 0);

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::turn_offset_x = 0;
std::atomic<double> Robot::turn_offset_y = 0;

double Robot::offset_back = 2.875;
double Robot::offset_middle = 5.0;
double Robot::wheel_circumference = 2.75 * 3.1415926;
double pi = 3.141592653589793238;

int counter_global = 0;

void Robot::print(nlohmann::json msg) {
	x = (float)x + 1;
    string msgS = msg.dump();
	//lcd::print(1, "Received %s %f", msg.dump(), (float)x);
	std::size_t found = msgS.find(",");
	double angle = std::stod(msgS.substr(1,found-1));
	double depth = std::stod(msgS.substr(found+1,msgS.size()-found-1));
	//lcd::print(5, "Degree %s", msgS.substr(1,found-1), (float)x);
	//lcd::print(6, "Depth %s", msgS.substr(found+1,msgS.size()-found-2), (float)x);
	double mogo_x = x + depth*(sin(angle * pi / 180));
	double mogo_y = y + depth*(cos(angle * pi / 180));
	//lcd::print(5, "depth %s", to_string(depth));
	//lcd::print(6, "angle %s", to_string(angle));
	//lcd::print(5, "Mogo_x %s", to_string(mogo_x), (float)x);
	//lcd::print(6, "Mogo_y %s", to_string(mogo_y), (float)x);
    std::vector<double> pos = {mogo_x, mogo_y+10, 0};
    lcd::print(5, "DONE %d", mogo_x);
    lcd::print(6, "DONE %d", mogo_y);
    move_to(pos);
	//mec_wrapper(0, 0, turn/10);
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
		bool pressed = master.get_digital(DIGITAL_R1);
		bool pressed2 = master.get_digital(DIGITAL_R2);
		/*
		if (pressed){
			roller=100;
		}
		else if(pressed2){
			roller = -100;
		}
		else{
			roller = 0;
		}
		*/
        mecanum(power, strafe, turn);
        delay(5);
	}
}

// void Robot::mecanum(int power, int strafe, int turn) {

// 	int powers[] {
// 		power + strafe + turn,
// 		power - strafe - turn,
// 		power - strafe + turn, 
// 		power + strafe - turn
// 	};

// 	int max = *max_element(powers, powers + 4);
// 	int min = abs(*min_element(powers, powers + 4));

// 	double true_max = double(std::max(max, min));
// 	double scalar = (true_max > 127) ? 127 / true_max : 1;
	
// 	FLT = 0*(power + strafe + turn) * scalar;
// 	FLB = 0*(power + strafe + turn) * scalar;

// 	FRT = (power - strafe - turn) * scalar;
// 	FRB = (power - strafe - turn) * scalar;
// 	BLT = (power - strafe + turn) * scalar;
// 	BLB = (power - strafe + turn) * scalar;
// 	BRT = (power + strafe - turn) * scalar;
// 	BRB = (power + strafe - turn) * scalar;
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
	scalar = 1;
	

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
    while (true) {
        double cur_phi = IMU.get_rotation() * 3.1415926 / 180;
        double dphi = cur_phi - last_phi;

        double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
        double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;
        /* Calculate how much the encoders have turned as a result of turning ONLY in order to
        isolate readings representing lateral or axial movement from readings representing
        turning in place */

        turn_offset_x = (float)turn_offset_x + cur_turn_offset_x;
        turn_offset_y = (float)turn_offset_y + cur_turn_offset_y;

        double cur_y = ((LE.get_value() - turn_offset_y) - (RE.get_value() + turn_offset_y)) / -2;
        double cur_x = BE.get_value() - turn_offset_x;

        double dy = cur_y - last_y;
        double dx = cur_x - last_x;

        double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
        double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);
        /* Apply rotation matrix to dx and dy to calculate global_dy and global_dx. Is required because if the Robot moves
        on an orientation that is not a multiple of 90 (i.e. 22 degrees), x and y encoder values do not correspond
        exclusively to either x or y movement, but rather a little bit of both */

        y = (float)y + global_dy;
        x = (float)x + global_dx;

        lcd::print(1,"Y: %f - X: %f", (float)y, (float)x, IMU.get_rotation());
		
		lcd::print(2, "IMU value: %f", IMU.get_heading());
		lcd::print(3, "LE: %d RE: %d", LE.get_value(), RE.get_value());
		lcd::print(4, "BE: %d", BE.get_value());
        last_y = cur_y;
        last_x = cur_x;
        last_phi = cur_phi;

        delay(5);
        /* All of these calculations assume that the Robot is moving in a straight line at all times. However, while this
        is not always the case, a delay of 5 milliseconds between each calculation makes dx and dy (distance traveled on
        x and y axes) so small that any curvature is insignificant. */
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
void Robot::move_to(std::vector<double> pose) 
{
    double new_y = pose[0];
    double new_x = pose[1];
    double heading = pose[2];


    std::deque<double> motion;

    double y_error = new_y - y;
    double x_error = -(new_x - x);
    int coefficient = 0;
    double last_x = x;
    double last_y = y;
    std::string powered = "intakes";

    int time = 0;

    double heading2 = (heading < 0) ? heading + 360 : heading - 360;
    double imu_error = -(IMU.get_rotation() - heading);
    /* Calculate inverse headings (i.e. 1 deg = -359 deg), then find which heading is closer to current heading. For
    example, moving to -358 deg would require almost a full 360 degree turn from 1 degree, but from its equivalent of -359
    deg, it only takes a minor shift in position */

    while (abs(y_error) > 10 || abs(x_error) > 10 || abs(imu_error) > 1)
    { /* while Robot::y, Robot::x and IMU heading are all more than the specified margin away from the target */

        if ((int)motion.size() == 10) motion.pop_front();
        motion.push_back(abs(last_x - x) + abs(last_y - y));
        double sum = 0;
        for (int i = 0; i < motion.size(); i++) sum += motion[i];
        double motion_average = sum / 10;

        last_x = x;
        last_y = y;

        double phi = (IMU.get_rotation()) * pi / 180;
        double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
        double turn = turn_PD.get_value(imu_error);
        mecanum(power, strafe, turn);
        /* Using our PD objects we use the error on each of our degrees of freedom (axial, lateral, and turning movement)
        to obtain speeds to input into Robot::mecanum. We perform a rotation matrix calculation to translate our y and x
        error to the same coordinate plane as Robot::y and Robot::x to ensure that the errors we are using are indeed
        proportional/compatible with Robot::y and Robot::x */

        imu_error = -(IMU.get_rotation() - heading);
        y_error = new_y - y;
        x_error = -(new_x - x);
        /* Recalculating our error by subtracting components of our current position vector from target position vector */

        delay(5);
        time += 5;
    }
    reset_PD();
    lcd::print(6, "DONE %d", counter_global);
    counter_global ++;
    brake("stop");
}



