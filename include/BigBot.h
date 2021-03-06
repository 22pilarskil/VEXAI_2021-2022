#ifndef _BigBot
#define _BigBot
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include "system/json.hpp"
#include "utils/PD.h"

#include <atomic>
using namespace pros;
using namespace std;
class BigBot{
	public:
		static Controller master;
		static PD power_PD;
		static PD strafe_PD;
		static PD turn_PD;

		static Motor FLT;
		static Motor FRT;
		static Motor BLT;
		static Motor BRT;
		static Motor FLB;
		static Motor FRB;
		static Motor BLB;
		static Motor BRB;
		static Motor flicker;
		static Motor angler;
		static Motor conveyor;
		static Motor lift;

		static ADIEncoder LE;
		static ADIEncoder RE;
		static ADIEncoder BE;
		static ADIAnalogIn angler_pot;
		static ADIAnalogIn lift_pot;
		static ADIDigitalOut angler_piston;
		static ADIDigitalOut lift_piston;
		static ADIUltrasonic ring_ultrasonic;
		static Gps gps;
		static Imu IMU;
		static Distance angler_dist;
		static Distance mogo_dist;

		static std::atomic<double> x;
		static std::atomic<double> y;
		static std::atomic<double> heading;
		static std::atomic<double> new_x;
		static std::atomic<double> new_y;
		static std::atomic<double> new_heading;

		static std::atomic<double> x_gps;
		static std::atomic<double> y_gps;
		static std::atomic<double> heading_gps;
		static std::atomic<double> new_x_gps;
        static std::atomic<double> new_y_gps;
        static std::atomic<double> new_heading_gps;
		static std::atomic<double> last_x_gps;
		static std::atomic<double> last_y_gps;
		static std::atomic<double> last_heading_gps;

        static std::atomic<int> stagnant;
        static std::atomic<double> drive_temp;

        static std::string mode;
        static bool stop;

		static double offset_back;
		static double offset_middle;
		static double wheel_circumference;
		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

		static void receive_fps(nlohmann::json msg);
		static void ring_receive(std::vector<float> det);
		static void mogo_receive(std::vector<float> det);
		static void receive_data(nlohmann::json msg);
		static void dummy(nlohmann::json msg);

		static void drive(void *ptr);
		static void check_depth(void *ptr);
		static void depth_angler(void *ptr);
		static void imu_clamp(void *ptr);
		static void fps(void *ptr);
        static void gps_fps(void *ptr);
        static void move_to_gps(void *ptr);
        static void reset(void *ptr);
		static void motor_temperature(void *ptr);

		static void reposition(void *ptr);


		static void move_to(void *ptr);
		static void is_moving_gps(void *ptr);
		static void display(void *ptr);
		static void start_task(std::string name, void (*func)(void *));
		static bool task_exists(std::string name);
		static void kill_task(std::string name);

		static void stay();

		static void mecanum(int power, int strafe, int turn, int max_power=127);
};
#endif
