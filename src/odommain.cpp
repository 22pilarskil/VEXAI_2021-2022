#include <vector>
#include <iostream>
#include <chrono>
//#include <ctime>
#include "Constants.hpp"
#include "mathutils.hpp"

using namespace std;

//void update();
void get_pos_estimate();
void setPose(struct Position _pos);
double encoder_to_distance(double ticks);

struct Position
{
  double x;
  double y;
  double heading; 
};
Position pos;

int main()
{
  get_pos_estimate();
  cout << "X:" << pos.x << "\nY:" << pos.y << "\nHeading:" << pos.heading;
}

double v_x = 0;
double v_y = 0;
double v_angular = 0;
double orientation = 0;
double rotation = 0;
double erPosLast = 0;
double elPosLast = 0;
double ebPosLast = 0;
auto time = chrono::system_clock::now(); //I don't know how time objects work, can someone please figure out delta time? 
					//(it should update with every call to the get_pos_estimate() func)

// void update() {
// 	get_pos_estimate();
// 	x = pos[0];
// 	y = pos[1];
// 	heading = pos[2];
// }

void get_pos_estimate() {
	double prevX = pos.x;
	double prevY = pos.y;
	double prevHeading = pos.heading;
	auto prevTime = time;
  time = chrono::system_clock::now();
  chrono::duration<double> elapsed_time = *time - *prevTime;
  cout << elapsed_time.count() << endl;

	double erPos = 0;
  //encoder_to_distance(Robot.getInstance().er.getCurrentPosition());
	double elPos = 0;
  //-encoder_to_distance(Robot.getInstance().el.getCurrentPosition());
	double ebPos = 0;
  //encoder_to_distance(Robot.getInstance().eb.getCurrentPosition());

	// Log.d(TAG, "er pos (inches): " + erPos);
	// Log.d(TAG, "el pos (inches): " + elPos);
	// Log.d(TAG, "eb pos (inches): " + ebPos);

	double dr = erPos-erPosLast;
	double dl = elPos-elPosLast;
	double db = ebPos-ebPosLast;

	double dHeading = (dr-dl)/(Constants::axialToCenterDist*2) ; //gets heading based on movement of right and left deadwheels
	double dx = db; //movement of back dead wheel to see movement on x-axis
	double dy = (dr+dl)/2.0;//average of left and right deadwheel movement to get overall y position of bot

	// Log.d(TAG, "dYaw: " + dHeading);
	// Log.d(TAG, "dx: " + dx);
	// Log.d(TAG, "dy: " + dy);

	pos.heading = mathutils::normalize(pos.heading+dHeading);

	double s,c;
	if (mathutils::equals(dHeading,0)) {
		s = 1-dHeading*dHeading/6.0;
		c = dHeading/2.0;
	} else {
		s = sin(dHeading)/dHeading;
		c = (1-cos(dHeading))/dHeading;
	}

	double dfX = dx*s-dy*c; //finds local delta x and y
	double dfY = dx*c+dy*s;

	double dxp = dfX*cos(pos.heading)-dfY*sin(pos.heading);//accounts for change in local deltas into the global deltas
	double dyp = dfX*sin(pos.heading)+dfY*cos(pos.heading);

	pos.x += dxp;
	pos.y += dyp;

	orientation += dHeading;
	rotation = orientation/(2*M_PI);
	if (rotation > 0) rotation = floor(rotation);
	else if (rotation < 0) rotation = ceil(rotation);

  // v_x = (x-prevX)/elapsed_time.count();
	// v_y = (y-prevY)/elapsed_time.count();
	// v_angular = (heading-prevHeading)/elapsed_time.count();

	// Log.d(TAG, "vx: " + v_x);
	// Log.d(TAG, "vy: " + v_y);
	// Log.d(TAG, "vang: " + v_angular);

	// Log.d(TAG, "rotation: " + rotation);
	// Log.d(TAG, "orientation: " + orientation);

	// Log.d(TAG, "x: " + x);
	// Log.d(TAG, "y: " + y);
	// Log.d(TAG, "heading: " + heading);

	erPosLast = erPos;
	elPosLast = elPos;
	ebPosLast = ebPos;
}

void setPose(struct Position _pos) {
	pos = _pos;
	pos.x = _pos.x;
	pos.y = _pos.y;
	pos.heading = _pos.heading;
	orientation = _pos.heading;
	rotation = 0;
	erPosLast = 0;
	elPosLast = 0;
	ebPosLast = 0;
}

double encoder_to_distance(double ticks) {
	double circumference = M_PI*Constants::dwDiameter;
	double distance = circumference * (ticks/Constants::dwTicksPerRev);
	return distance;
}
