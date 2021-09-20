#include "main.h"
#include "PD.h"
//using namespace pros

PD::PD (double p, double d, double min)
{
  kp = p;
  kd = d;
  min_speed = min;
  
  prev_error = 0; 
  prev_time = 0;
}

double PD::getValue(double error)
{
  int time = millis();
  int delta_time = time - prev_time;
  
  double error_derivative = (error - prev_error) / delta_time;
  
  prev_error = error;
  prev_time = time;
  double speed = (kp * error) + (kd * error_derivative);
  
  return (abs(speed) > min_speed) ? speed : (speed > 0) ? min_speed : -min_speed;
}
