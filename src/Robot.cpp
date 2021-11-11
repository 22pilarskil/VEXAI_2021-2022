
#include "main.h"
#include "Robot.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include <map>
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <bits/stdc++.h> 
using namespace pros;
using namespace std;

#define TO_RAD(n) n * M_PI / 180;

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FLT(2, true); //front left top
Motor Robot::FLB(1); //front left bottom
Motor Robot::FRT(9); //front right top
Motor Robot::FRB(10, true); //front right bottom
Motor Robot::BRT(20, true); //back right top
Motor Robot::BRB(19); //back right bottom
Motor Robot::BLT(11); //back left top
Motor Robot::BLB(12, true); //back left bottom
Motor Robot::roller(18); //mechanism for ascending rings
Imu Robot::IMU(15);
ADIEncoder Robot::LE(5, 6);
ADIEncoder Robot::RE(3, 4);
ADIEncoder Robot::BE(7, 8);

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::turn_offset_x = 0;
std::atomic<double> Robot::turn_offset_y = 0;

double Robot::offset_back = 2.875;
double Robot::offset_middle = 5.0;
double Robot::wheel_circumference = 2.75 * M_PI;

void Robot::print(nlohmann::json msg) {
	x = (float)x + 1;
	lcd::print(1, "Received %s %f", msg.dump(), (float)x);
}

void Robot::drive(void *ptr) {
	
	bool pressed = master.get_digital(DIGITAL_R1);
	bool move = false;
	while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);
		bool pressed = master.get_digital(DIGITAL_R1);
		bool pressed2 = master.get_digital(DIGITAL_R2);

		if (pressed){
			roller=100;
		}
		else if(pressed2){
			roller = -100;
		}
		else{
			roller = 0;
		}
        mecanum(power, strafe, turn);
		if(pressed == true) {
			move = !move;
		}
		if(move == true) {
			roller = 20;
		}
        delay(5);
	}
}

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
	
	FLT = 0*(power + strafe + turn) * scalar;
	FLB = 0*(power + strafe + turn) * scalar;

	FRT = (power - strafe - turn) * scalar;
	FRB = (power - strafe - turn) * scalar;
	BLT = (power - strafe + turn) * scalar;
	BLB = (power - strafe + turn) * scalar;
	BRT = (power + strafe - turn) * scalar;
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
    while (true) {
        double cur_phi = TO_RAD(IMU.get_rotation());
        double dphi = cur_phi - last_phi;

        double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
        double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;
        /* Calculate how much the encoders have turned as a result of turning ONLY in order to
        isolate readings representing lateral or axial movement from readings representing
        turning in place */

        turn_offset_x = (float)turn_offset_x + cur_turn_offset_x;
        turn_offset_y = (float)turn_offset_y + cur_turn_offset_y;

        double cur_y = ((LE.get_value() - turn_offset_y) + (RE.get_value() + turn_offset_y)) / 2;
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

        lcd::print(1, ("Y: %f - X: %f - IMU value: %f\n", (float)y, (float)x, IMU.get_rotation()));
		//lcd::print(2, IMU.get_rotation);
		lcd::print(2, ("uh"));


        last_y = cur_y;
        last_x = cur_x;
        last_phi = cur_phi;

        delay(5);
        /* All of these calculations assume that the Robot is moving in a straight line at all times. However, while this
        is not always the case, a delay of 5 milliseconds between each calculation makes dx and dy (distance traveled on
        x and y axes) so small that any curvature is insignificant. */
    }
}