#include "main.h"
#include "Robot.h"
#include "FifteenInch.h"
#include "system/Serial.h"
using namespace pros;

/* Creates all tasks required for our Robot's driver control period */

void opcontrol() {

	lcd::initialize();
	serial_initialize();

	delay(100);
	//FifteenInch::start_task("DRIVE", FifteenInch::drive);
	//FifteenInch::start_task("GPS INIT", FifteenInch::gps_initialize);
	//FifteenInch::start_task("GPS Test", FifteenInch::gps_test);

	lcd::print(1, "sent");
	lib7405x::Serial::Instance()->onReceive("whole_data",FifteenInch::receive_data);
	lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#@#");
	// serial_initialize();
	// lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#camera#l515_back#mode#false#@#");
	// //Robot::start_task("GPS", Robot::gps_fps);
	// Robot::start_task("DRIVE", Robot::drive);
	// Robot::start_task("CONTROLLER", Robot::controller_print);

	// Robot::IMU.reset();
	// delay(2500);
	// Robot::start_task("FPS", Robot::fps);
	// Robot::start_task("DISPLAY", Robot::display);
	// lib7405x::Serial::Instance()->onReceive("fps", Robot::receive_fps);
	// lib7405x::Serial::Instance()->onReceive("mogo", Robot::receive_mogo);
	// lib7405x::Serial::Instance()->onReceive("ring", Robot::receive_ring);
	// //Robot::start_task("MOVETO", Robot::move_to);
	// Robot::start_task("IMU", Robot::imu_clamp);

}
