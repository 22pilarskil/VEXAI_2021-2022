#include "main.h"

class PD{
	public:
		double kp;
		double kd;
		double minspeed;
		int counter;
		/* PD object presets */

		double prev_error;
		int prev_time;
		double D;
		int counter_reset;
		/* Variables required for each PD calculation */

		PD(double p, double d, double min = 0, int counter_ = 100);
		double get_value(double error);
		void reset();
};