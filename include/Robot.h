#include "main.h"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include "system/json.hpp"
using namespace pros;

class Robot{
	public:
		static Controller master;
		static Motor FLT;
		static Motor FRT;
		static Motor BLT;
		static Motor BRT;
		static Motor FLB;
		static Motor FRB;
		static Motor BLB;
		static Motor BRB;
		static Motor FL;
		static Motor BL;
		static Motor FR;
		static Motor BR;
		static std::atomic<double> x;

		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

		static void mecanum(int power, int strafe, int turn);
		static void drive(void *ptr);
		static void print(nlohmann::json msg);

		static void start_task(std::string name, void (*func)(void *));
		static bool task_exists(std::string name);
		static void kill_task(std::string name);
};