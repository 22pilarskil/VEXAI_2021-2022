#include "main.h"
#include "BigBot.h"
#include "SmallBot.h"
#include "system/Serial.h"
using namespace pros;
/* Creates all tasks required for our BigBot's driver control period */


std::string bot = "big";
int slot = 1;


void opcontrol() {

	lcd::initialize();
	serial_initialize();


	if (bot.compare("big") == 0){

		std::string mode = "mogo";
		BigBot::IMU.reset();
		if (slot == 1){

			delay(2500);
			BigBot::start_task("IMU", BigBot::imu_clamp);
			if (mode.compare("ring") == 0){
				BigBot::mode = "ring";
				lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
			}
			if (mode.compare("mogo") == 0){
				BigBot::mode = "mogo";
				lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_back#@#");
			}
			lib7405x::Serial::Instance()->onReceive("whole_data", BigBot::receive_data);
			lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");
			BigBot::start_task("MOVETO", BigBot::move_to);
			BigBot::stay(); //stops the bot moving
			BigBot::start_task("GPS", BigBot::gps_fps);
			BigBot::start_task("MOVETOGPS", BigBot::move_to_gps);
			BigBot::start_task("RESET", BigBot::reset);

		}

		if (slot == 2){
			delay(2500);
			BigBot::start_task("IMU", BigBot::imu_clamp);
			BigBot::start_task("FPS", BigBot::fps);
			lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
			BigBot::start_task("DRIVE", BigBot::drive);
			lib7405x::Serial::Instance()->onReceive("whole_data", BigBot::dummy);
		}

		if (slot == 3){
			delay(2500);
			BigBot::start_task("IMU", BigBot::imu_clamp);
			BigBot::start_task("MOVETO", BigBot::move_to);
			BigBot::new_x = 300;
			BigBot::new_y = 300;
			BigBot::heading = 45;

		}

		lib7405x::Serial::Instance()->onReceive("fps", BigBot::receive_fps);
		BigBot::start_task("TEMPERATURE", BigBot::motor_temperature);
		BigBot::start_task("GPS", BigBot::gps_fps);
		BigBot::start_task("FPS", BigBot::fps);
		BigBot::start_task("DISPLAY", BigBot::display);
		BigBot::start_task("MOVING", BigBot::is_moving_gps);
		BigBot::start_task("REPOSITION", BigBot::reposition);

	}

	if (bot.compare("small") == 0){

		if (slot == 1){
			lib7405x::Serial::Instance()->onReceive("whole_data",SmallBot::receive_data);
			lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#@#");
			serial_initialize();
			lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#camera#l515_back#mode#false#@#");
		}

		if (slot == 2){
			delay(100);
			SmallBot::start_task("gps",SmallBot::gps_initialize);
			SmallBot::start_task("DRIVE", SmallBot::drive);
			SmallBot::start_task("GPS INIT", SmallBot::gps_initialize);
			SmallBot::start_task("GPS Test", SmallBot::gps_test);
		}

	}


}
