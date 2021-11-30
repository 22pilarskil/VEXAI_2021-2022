#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {

	lcd::initialize();
	serial_initialize();
	Robot::start_task("DRIVE", Robot::drive);
	Robot::IMU.reset();
	delay(3000);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("MOVETO", Robot::move_to);
	lib7405x::Serial::Instance()->onReceive("mogo", Robot::receive);
	// delay(1000);



}
