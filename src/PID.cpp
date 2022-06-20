//credit to zayn rekhi, 7405k

#include "main.h"
#include "PID.h"
#include <cmath>
using namespace pros;
PID::PID(double p, double i, double d, double i_bound) {
    kp = p;
    ki = i;
    kd = d;
    integral_bound = i_bound;

    error_sum = 0;      //Needed for Integral
    prev_time = 0;  //Needed for Derivative
    prev_error = 0;
}

/**
 * @brief Calculates the current speeds based on the error term
 * @param error - The distance from the current position and the target position
 * @return Sum of P, I, and D calculations: SPEED
 */

double PID::get_value(double error) {
    int time = pros::millis();
    error_sum += error;

    double delta_error = error - prev_error;
    int delta_time = time - prev_time;
    double derivative_of_error = delta_error / (double)delta_time;

    double p_calc = kp * error;
    double i_calc = ki * error_sum;
    double d_calc = kd * derivative_of_error;

    if(std::abs(error) > integral_bound && integral_bound != 0) {
      i_calc = 0;
      error_sum = 0;
    }


    prev_error = error;
    prev_time = time;
    lcd::print(4, "%d", p_calc+i_calc+d_calc);
    return p_calc + i_calc + d_calc;
}


double PID::get_value2(double error) {
  int time = pros::millis();
	int delta_time = time - prev_time;
	/* Allow for PID to take into account imperfect loop times- delay(5) does not always delay 5 milliseconds */

  double derivative_of_error = (error - prev_error) / delta_time;
  //

	prev_error = error;
	prev_time = time;

	double speed = (kp * error) + (kd * derivative_of_error);
  return speed;
}

/**
 * @brief resets running pid values for next run
 */
void PID::reset() {
    error_sum = 0;
    prev_time = 0;
    prev_error = 0;
}
