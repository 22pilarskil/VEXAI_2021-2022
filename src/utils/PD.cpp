#include "main.h"
#include "PD.h"
using namespace pros;

PD::PD(double p, double d, double min, int counter_) {
	kp = p;
	kd = d;
	minspeed = min;
	counter = counter_;

	prev_error = 0;
	prev_time = 0;
	D = 0;
	counter_reset = counter_;
}

double PD::get_value(double error) {
	int time = millis();
	int delta_time = time - prev_time;

    double derivative_of_error = (error - prev_error) / delta_time;

	prev_error = error;
	prev_time = time;
	counter++;

	double speed = (kp * error) + (kd * derivative_of_error);
	double coefficient = (std::min(100, counter))/100;
	return coefficient *(abs(speed) > minspeed) ? speed : (speed > 0) ? minspeed : -minspeed;
}

void PD::reset() {
	counter = counter_reset;
}