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

// same as other function excepts slows down before stop_error and stops once error is within 0 and stop_error. starts slowing down at slowdown_error using arbetrary curve
double PD::get_value(double error, double stop_error, double slowdown_error) {
	int time = millis();
	int delta_time = time - prev_time;

    double derivative_of_error = (error - prev_error) / delta_time;

	prev_error = error;
	prev_time = time;
	counter++;

	double speed = (kp * error) + (kd * derivative_of_error);

	if (error <= stop_error) {
		return 0;
	} else if(error <= slowdown_error) {
		//cool curve between points (0, min_speed) and (slowdown_error, 1) where the x value is distance and y value is speed
		speed = -(1 - minspeed)*(sqrt((1.0/slowdown_error)*speed) - 1)^2 + 1 //the 1 at the end is presumably the max speed
	}

	//this coefficient could be removed to optimize speed
	double coefficient = (std::min(100, counter))/100;
	return coefficient *(abs(speed) > minspeed) ? speed : (speed > 0) ? minspeed : -minspeed;
}

void PD::reset() {
	counter = counter_reset;
}