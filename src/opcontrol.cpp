#include "main.h"
#include "Robot.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	delay(100);
	Robot::start_task("DRIVE", Robot::drive);

}
