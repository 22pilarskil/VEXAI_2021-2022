#include "main.h"
#include "Robot.h"
#include "system/Serial.h"
using namespace pros;
/* Creates all tasks required for our Robot's driver control period */
void opcontrol() {

	std::string mode = "mogo";
	int slot = 1;
	lcd::initialize();
<<<<<<< Updated upstream
	serial_initialize();
	Robot::IMU.reset();
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
		lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");
		Robot::start_task("MOVETO", Robot::move_to);
		Robot::stay(); //stops the bot moving
		Robot::start_task("GPS", Robot::gps_fps);
		Robot::start_task("MOVETOGPS", Robot::move_to_gps);
		Robot::start_task("RESET", Robot::reset);

	}

	if (slot == 2){
		delay(2500);
		Robot::start_task("IMU", Robot::imu_clamp);
		Robot::start_task("FPS", Robot::fps);
		lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
		Robot::start_task("DRIVE", Robot::drive);
		lib7405x::Serial::Instance()->onReceive("whole_data", Robot::dummy);
	}

	if (slot == 3){
		delay(2500);
		Robot::start_task("IMU", Robot::imu_clamp);
		Robot::start_task("MOVETO", Robot::move_to);
		Robot::new_x = 300;
		Robot::new_y = 300;
		Robot::heading = 45;

	}

	lib7405x::Serial::Instance()->onReceive("fps", Robot::receive_fps);
	Robot::start_task("TEMPERATURE", Robot::motor_temperature);
	Robot::start_task("GPS", Robot::gps_fps);
	Robot::start_task("FPS", Robot::fps);
	Robot::start_task("DISPLAY", Robot::display);
	Robot::start_task("MOVING", Robot::is_moving_gps);
	Robot::start_task("REPOSITION", Robot::reposition);




=======
	//serial_initialize();

	delay(100);
	lcd::print(1, "Start");
	FifteenInch::start_task("gps",FifteenInch::gps_initialize);
	FifteenInch::start_task("DRIVE", FifteenInch::dummy);
	//FifteenInch::start_task("GPS INIT", FifteenInch::gps_initialize);
	//FifteenInch::start_task("GPS Test", FifteenInch::gps_test);

	// lcd::print(1, "sendding");
	// lib7405x::Serial::Instance()->onReceive("whole_data",FifteenInch::receive_data);
	// lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#@#");
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
>>>>>>> Stashed changes

}
