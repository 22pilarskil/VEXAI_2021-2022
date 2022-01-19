#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	serial_initialize();
	//Robot::start_task("GPS", Robot::gps_fps);
	Robot::start_task("DRIVE", Robot::drive);
	// Robot::IMU.reset();
	// delay(2500);
	// Robot::start_task("FPS", Robot::fps);
	// Robot::start_task("MOVETO", Robot::move_to);
	// lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#");
	// lib7405x::Serial::Instance()->onReceive("mogo", Robot::receive_mogo);
	// delay(1000);



}
