#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	serial_initialize();
	delay(100);
	serial_initialize();
	// Robot::start_task("DRIVE", Robot::drive);
	Robot::IMU.reset();
	// while (true){
	// 	lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "header#stop");
	// 	delay(1000);
	// }
	delay(3000);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("MOVETO", Robot::move_to);
	lib7405x::Serial::Instance()->onReceive("header", Robot::receive);
	// delay(1000);



}
