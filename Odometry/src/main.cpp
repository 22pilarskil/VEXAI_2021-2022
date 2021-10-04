#include "main.h"
#include <math.h>
#include <vector>
#include "mathutils.h"
#include "Constants.h"
#include <vector>
#include <iostream>
#include <chrono>
//#include <ctime>
#include "Constants.hpp"
#include "mathutils.hpp"

using namespace std;

void update();
vector<double> get_pos_estimate();
void setPose(vector<double> _pos);
double encoder_to_distance(double ticks);

int main()
{
  vector<double> pos_vector = get_pos_estimate();
  cout << "X:" << pos_vector[0] << "\nY:" << pos_vector[1] << "\nHeading:" << pos_vector[2];
}

vector<double> pos = {0, 0, 0};
double x = 0;
double y = 0;
double heading = 0;
double v_x = 0;
double v_y = 0;
double v_angular = 0;
double orientation = 0;
double rotation = 0;
double erPosLast = 0;
double elPosLast = 0;
double ebPosLast = 0;
auto time = chrono::system_clock::now();

void update() {
	pos = get_pos_estimate();
	x = pos[0];
	y = pos[1];
	heading = pos[2];
}

vector<double> get_pos_estimate() {
	double prevX = x;
	double prevY = y;
	double prevHeading = heading;
	auto prevTime = *time;
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

	heading = mathutils::normalize(heading+dHeading);

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

	double dxp = dfX*cos(heading)-dfY*sin(heading);//accounts for change in local deltas into the global deltas
	double dyp = dfX*sin(heading)+dfY*cos(heading);

	x += dxp;
	y += dyp;

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

	return vector<double> {x,y,heading};
}

void setPose(vector<double> _pos) {
	pos = _pos;
	x = pos[0];
	y = pos[1];
	heading = pos[2];
	orientation = pos[2];
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

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_mtr = left;
		right_mtr = right;
		pros::delay(20);
	}
}
