
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


std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

std::atomic<double> Robot::x = 0;

Controller Robot::master(E_CONTROLLER_MASTER);
Motor Robot::FLT(2); //front left top
Motor Robot::FLB(1, true); //front left bottom
Motor Robot::FRT(9); //front right top
Motor Robot::FRB(10, true); //front right bottom
Motor Robot::BRT(20); //back right top
Motor Robot::BRB(19, true); //back right bottom
Motor Robot::BLT(11); //back left top
Motor Robot::BLB(12, true); //back left botto

void Robot::print(nlohmann::json msg) {
	x = (float)x + 1;
	lcd::print(1, "Received %s %f", msg.dump(), (float)x);
}

void Robot::drive(void *ptr) {
	while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        mecanum(power, strafe, turn);
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
	

	FLT = -1*(power + strafe + turn) * scalar;
	FLB = -1*(power + strafe + turn) * scalar;
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