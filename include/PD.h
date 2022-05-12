#ifndef _PD
#define _PD
#include "main.h"

class PD {
	public:
		double kp;
		double kd;
		double minspeed;
		int counter;

		double prev_error;
		int prev_time;
		double D;
		int counter_reset;

		PD(double p, double d, double min = 0, int counter_ = 100);
		double get_value(double error);
		double get_value(double error, double stop_error, double slowdown_error);
		void reset();
};

#endif