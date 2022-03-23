#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	delay(100);
	serial_initialize();
	Robot::start_task("DRIVE", Robot::drive);
	delay(100);
	//Robot::start_task("GPSFPS", Robot::gps_fps);
	//Robot::start_task("ISMOVING", Robot::is_moving_gps);
	Robot::start_task("GPSFPS", Robot::gps_fps);
	Robot::start_task("ISMOVINGPRINT", Robot::is_moving_print);


}

