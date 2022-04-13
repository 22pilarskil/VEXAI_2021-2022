#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	std::string mode = "full";
	lcd::initialize();
	serial_initialize();
	delay(2500);
	Robot::start_task("IMU", Robot::imu_clamp);


	lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#continue_ring#true#");

	if (mode.compare("ring") == 0){
		Robot::mode = "ring";
		lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
	}
	if (mode.compare("full") == 0){
		Robot::mode = "mogo";
		lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_back#@#");
	}

	//Robot::start_task("GPS", Robot::gps_fps);
	//Robot::start_task("DRIVE", Robot::drive);
	Robot::start_task("CONTROLLER", Robot::controller_print);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	lib7405x::Serial::Instance()->onReceive("fps", Robot::receive_fps);
	lib7405x::Serial::Instance()->onReceive("whole_data", Robot::receive_data);
	Robot::start_task("MOVETO", Robot::move_to);

}

