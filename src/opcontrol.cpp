#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {
	lcd::initialize();
	serial_initialize();
	delay(100);
	Robot::start_task("GPS", Robot::gps_fps);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("MOVING", Robot::is_moving_gps);
	lib7405x::Serial::Instance()->onReceive("whole_data", Robot::dummy);
	lib7405x::Serial::Instance()->onReceive("fps", Robot::receive_fps);

	/**




	std::string mode = "mogo";
	int slot = 1;
	lcd::initialize();
	serial_initialize();
	delay(100);

	if (slot == 1){
		delay(2500);
		Robot::start_task("IMU", Robot::imu_clamp);

		if (mode.compare("ring") == 0){
			Robot::mode = "ring";
			lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
		}
		if (mode.compare("mogo") == 0){
			Robot::mode = "mogo";
			lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_back#@#");
		}

		lib7405x::Serial::Instance()->onReceive("whole_data", Robot::receive_data);
		Robot::start_task("MOVETO", Robot::move_to);
		Robot::stay(); //stops the bot moving
		Robot::start_task("GPS", Robot::gps_fps);
		//Robot::start_task("MOVETOGPS", Robot::move_to_gps);
		Robot::start_task("RESET", Robot::reset);

	}

	if (slot == 2){
		lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
		Robot::start_task("DRIVE", Robot::drive);
		lib7405x::Serial::Instance()->onReceive("whole_data", Robot::dummy);
	}

	if (slot == 3){
		Robot::start_task("RESET", Robot::reset);
	}

	lib7405x::Serial::Instance()->onReceive("fps", Robot::receive_fps);
	Robot::start_task("GPS", Robot::gps_fps);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("MOVING", Robot::is_moving_gps);
	**/



}
