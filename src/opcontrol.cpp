#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	lib7405x::Serial::Instance()->onReceive("header", Robot::print);
	delay(100);
	serial_initialize();
	Robot::start_task("DRIVE", Robot::drive);
	Robot::IMU.reset();
	delay(2000);
	Robot::start_task("FPS", Robot::fps);
	delay(1000);
	lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "Config not found!");



}
