#include "main.h"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <deque>
using namespace pros;

class Robot{
	public:
		static Controller master;
		static Motor FL;
		static Motor FR;
		static Motor BL;
		static Motor BR;
		static std::atomic<double> x;

		static std::map<std::string, std::unique_ptr<pros::Task>> tasks;

		static void mecanum(int power, int strafe, int turn);
		static void drive(void *ptr);

		static void start_task(std::string name, void (*func)(void *));
		static bool task_exists(std::string name);
		static void kill_task(std::string name);
};