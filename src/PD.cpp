#include "main.h"
#include "PD.h"
using namespace pros;

/* PD control is an algorithm designed to help our Robot make controlled stops. Since VEX robots have a large mass, they
carry a lot of momentum and stopping too suddenly can cause our Robot to jerk off course or overshoot its target position.
As a result, we want to disperse this momentum as we approach our target. We do this by using our error, or distance from
our current position to our target, to control our speed as a sum of two terms. The first term is directly proportional to
our error, and gets smaller as we approach our target. The second term is the derivative of our error over time, and is 
always a negative number as error decreases over time. Thus, both of these terms have a slowing effect on our Robot as we 
get closer to our target. For greater control, both of these terms are multiplied by tunable coefficients kp and kd which 
impact to what degree each term is expressed in our speed (i.e. a high kd would result in a slower robot as the derivative
term would be expressed to a greater degree) */

/**
 * @descL Constructor for our PID class/Initializes
 * variables that will be used by class later
 * @param p: Sets kp, the coefficient of our error term
 * @param d: Sets kd, the coefficient of our derivative term
 * @param min: Sets the minimum speed, designed to prevent Robot from grinding to a halt
 * @param counter_ Sets the counter
 */
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

/**
 * Calculates speed at which the Robot should still be moving at.
 * @param error: Error between target position and current position (calculated as difference between target and current
 	position vectors in function PID::get_value is being called in)
 * @return: Speed of motors associated with PID object
 */
double PD::get_value(double error) {
	int time = millis();
	int delta_time = time - prev_time;
	/* Allow for PID to take into account imperfect loop times- delay(5) does not always delay 5 milliseconds */

    double derivative_of_error = (error - prev_error) / delta_time;

	prev_error = error;
	prev_time = time;
	counter++;

	double speed = (kp * error) + (kd * derivative_of_error);
	double coefficient = (std::min(100, counter))/100;
	return coefficient *(abs(speed) > minspeed) ? speed : (speed > 0) ? minspeed : -minspeed;
}

/**
 * @desc: Resets counter back to counter_reset after each time we use PID in order to reset acceleration curve 
 */
void PD::reset() {
	counter = counter_reset;
}