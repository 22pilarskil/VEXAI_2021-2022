#include <vector>
#include <iostream>
#include <chrono>
//#include <ctime>
#include "Constants.h"
#include "mathutils.h"

struct Position
{
  double x;
  double y;
  double heading; 
};

double v_x = 0;
double v_y = 0;
double v_angular = 0;
double orientation = 0;
double rotation = 0;
double erPosLast = 0;
double elPosLast = 0;
double ebPosLast = 0;

void get_pos_estimate();
void setPose(struct Position _pos);
double encoder_to_distance(double ticks);
