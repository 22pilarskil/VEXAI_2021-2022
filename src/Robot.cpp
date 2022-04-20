
#include "main.h"
#include "Robot.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "PD.h"
#include <map>
#include <cmath>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
#include "GridMapper.cpp"
using namespace pros;
using namespace std;

Controller Robot::master(E_CONTROLLER_MASTER);
PD Robot::power_PD(.4, 0, 0);
PD Robot::strafe_PD(.4, 0, 0);
PD Robot::turn_PD(1.0, 0, 0);

Motor Robot::BRB(1, true);
Motor Robot::BRT(3);
Motor Robot::BLT(10, true);
Motor Robot::BLB(9);
Motor Robot::FLT(18, true);
Motor Robot::FLB(19);
Motor Robot::FRB(13, true);
Motor Robot::FRT(11);
Motor Robot::flicker(17);
Motor Robot::angler(20);
Motor Robot::conveyor(2);
Motor Robot::lift(8);

ADIEncoder Robot::LE({{16, 5, 6}});
ADIEncoder Robot::RE({{16, 1, 2}});
ADIEncoder Robot::BE({{16, 3, 4}});
ADIAnalogIn Robot::angler_pot({{16, 8}});
ADIAnalogIn Robot::lift_pot(3);
ADIDigitalOut Robot::angler_piston(2);
ADIDigitalOut Robot::lift_piston(1);
ADIUltrasonic Robot::ring_ultrasonic(5, 6);
Gps Robot::gps(4);
Imu Robot::IMU(12);
Distance Robot::angler_dist(21);
Distance Robot::mogo_dist(15);

const double inches_to_encoder = 41.669;
const double meters_to_inches = 39.3701;
const double pi = 3.141592653589793238;


std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::imu_val = 0;
std::atomic<double> Robot::new_x = 0;
std::atomic<double> Robot::new_y = 0;
std::atomic<double> Robot::heading = 0;
std::atomic<double> Robot::new_x_gps = 0;
std::atomic<double> Robot::new_y_gps = 0;
std::atomic<double> Robot::new_heading_gps = 0;
std::atomic<double> Robot::cur_x_gps = 0;
std::atomic<double> Robot::cur_y_gps = 0;
std::atomic<double> Robot::cur_heading_gps = 0;
std::atomic<double> Robot::last_x_gps = 0;
std::atomic<double> Robot::last_y_gps = 0;
std::atomic<double> Robot::last_phi_gps = 0;

std::atomic<bool> Robot::is_moving = false;

std::string Robot::mode;
bool Robot::stop = false;

double Robot::offset_back = 5.25;
double Robot::offset_middle = 7.625;
double Robot::wheel_circumference = 2.75 * pi;


std::atomic<bool> chasing_mogo = false;
std::atomic<double> turn_coefficient = 1;
std::atomic<bool> turn_in_place = true;
double seconds_per_frame = 0.20;
int failed_update = 0;
double last_heading = 0;
bool started = false;

GridMapper* gridMapper = new GridMapper();


std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;


std::vector<int> find_location(std::string sample, char find){
    std::vector<int> character_locations;
    for(int i = 0; i < sample.size(); i++){
        if (sample[i] == find){
            character_locations.push_back(i);
        }
    }
    return character_locations;
}


void Robot::receive_data(nlohmann::json msg)
{
    if (stop) return;
    started = true;
    vector<vector<float>> pred = get_pred(msg);
    if (mode.compare("mogo") == 0){
        vector<vector<float>> mogos = pred_id(pred, 0);
        for (vector<float> det : mogos){
            mogo_receive(det);
            continue;
        }
    }
    if (mode.compare("ring") == 0){
        vector<vector<float>> rings = pred_id(pred, 1);
        for (vector<float> det : rings){
            det[0] += 0.2;
            if (invalid_det(det, cur_x_gps, cur_y_gps, cur_heading_gps)) {
                continue;
            }
            ring_receive(det);
            lcd::print(4, "ring");
            return;
        }
        lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#");
    }
}


void Robot::dummy(nlohmann::json msg){
    vector<vector<float>> pred = get_pred(msg);
    int valid = 0;
    int invalid = 0;
    vector<vector<float>> rings = pred_id(pred, 1);
    for (vector<float> det : rings){
        det[0] += 0.2;
        if (invalid_det(det, last_x_gps, last_y_gps, last_phi_gps)) {
            invalid += 1;
        }
        else {
            valid += 1;
        }
    }
    lcd::print(6, "VALID %d INVALID %d", valid, invalid);
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");

}

vector<vector<float>> Robot::pred_id(vector<vector<float>> pred, int id)
{
    vector<vector<float>> tp;
    for(vector<float> curr:pred){
        if(curr[2]==id){
            tp.push_back(curr);
        }
    }
    return tp;
}

vector<vector<float>> Robot::get_pred(nlohmann::json msg){
    string s = msg.dump();
    s = s.substr(1, s.size()-2);
    string delimiter = "|";
    vector<vector<float>> pred;
    size_t pos = 0;
    string token;
    while ((pos = s.find(delimiter)) != string::npos) {
        token = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
        vector<float> read_curr;
        string delimiter2 = ",";

        size_t pos2 = 0;
        string token2;
        while ((pos2 = token.find(delimiter2)) != std::string::npos) {
            token2 = token.substr(0, pos2);
            token.erase(0, pos2 + delimiter2.length());
            read_curr.push_back(float(std::stod(token2)));
        }
        pred.push_back(read_curr);
    }
    return pred;
}

bool Robot::invalid_det(std::vector<float> det, double cur_x, double cur_y, double gps_heading)
{
    gps_heading = gps_heading*pi/180;//to radians
    double lidar_depth = det[0];
    double angle = det[1];
    double final_angle = gps_heading - angle*pi/180;
    final_angle < 0 ? final_angle = 360 + final_angle : final_angle = final_angle;//angle that connects ring and bot

    double ring_y = sin(final_angle)*lidar_depth+cur_y;

    double ring_x = cos(final_angle)*lidar_depth+cur_x;
    lcd::print(4, "%f, %f", (float)ring_x, (float)ring_y);
    lcd::print(5, "%f %f", (float)lidar_depth, (float)angle);


    double temp_dist = 12;//inches away from wall
    double min_wall_distance = (70.5-temp_dist) / meters_to_inches;

    double balance_threshold = 4; //how close we want to allow our bot to get to the balance in inches
    double balance_corner_y = (27 + balance_threshold) / meters_to_inches;
    double balance_corner_x = ((70.5-23) - balance_threshold) / meters_to_inches;

    // There is probably a more concise way to write this but it works so whatever.

    bool under_balance = (abs(ring_y) <= balance_corner_y) && (abs(ring_x) >= balance_corner_x);

    bool too_close_to_wall = abs(ring_y)>=min_wall_distance || abs(ring_x) >= min_wall_distance;

    if(!under_balance && !too_close_to_wall)
    {
      //define the line that connects the points in mx+b

      lcd::print(3, "%s", "LINE PROGRAM");

      bool east_intersects = false;
      bool west_intersects = false;
      bool north_intersects = false;
      bool south_intersects = false;
      double m = (ring_y-cur_y)/(ring_x-cur_x);
      double b = cur_y - m*cur_x;

      bool east_possible = (cur_x < balance_corner_x && ring_x > balance_corner_x) || (cur_x > balance_corner_x && ring_x<balance_corner_x);
      if(east_possible)
      {
        double east_intersect_val = m*balance_corner_x+b;//would be a y coord val
        east_intersects = east_intersect_val<= balance_corner_y && east_intersect_val >= -balance_corner_y;
      }
      bool west_possible = (cur_x < -balance_corner_x && ring_x > -balance_corner_x) || (cur_x > -balance_corner_x && ring_x < -balance_corner_x);
      if(west_possible)
      {
        double west_intersect_val = m*(-balance_corner_x)+b;
        west_intersects = west_intersect_val <= balance_corner_y && west_intersect_val >= -balance_corner_y;
      }
      bool north_possible = (cur_y < balance_corner_y && ring_y > balance_corner_y) || (cur_y > balance_corner_y && ring_y<balance_corner_y);
      if(north_possible)
      {
        double north_intersect_val = (balance_corner_y - b)/m;
        north_intersects = north_intersect_val >= balance_corner_x || north_intersect_val <= -balance_corner_x;
      }
      bool south_possible = (cur_y > -balance_corner_y && ring_y < -balance_corner_y)||(cur_y<-balance_corner_y && ring_y>-balance_corner_y);
      if(south_possible)
      {
        double south_intersect_val = (-balance_corner_y-b)/m;
        south_intersects = south_intersect_val >= balance_corner_x || south_intersect_val <= -balance_corner_x;
      }
      return north_intersects || south_intersects || east_intersects || west_intersects;

    }
    else{lcd::print(3, "%s", "NOT LINE PROGRAM"); return true;}
}

void Robot::mogo_receive(vector<float> det)
{
  //copy and pasted, changed to work with the attributes given by receive_data
  failed_update = 0;
  turn_in_place = false;
  chasing_mogo = true;
  flicker = 127;

  double angle_threshold = 1;
  turn_coefficient = 3;

  if (!task_exists("DEPTH")) start_task("DEPTH", Robot::check_depth);


  double lidar_depth = std::max((double)det[0], (double)0.2);
  double angle = det[1];
    
  //gridMapper->map([x, y, angle], {"mogo", [lidar_depth, angle]}); 
  //aight idk if this is right

  double coefficient;

  if (abs(angle) > angle_threshold) coefficient = 300 * lidar_depth * (0.20 / seconds_per_frame);
  else if (abs(angle) < angle_threshold && chasing_mogo == true) coefficient = 600;

  heading = imu_val + angle;
  new_y = y + coefficient * cos(heading / 180 * pi);
  new_x = x - coefficient * sin(heading / 180 * pi);

  delay(5);
  turn_coefficient = 1;
}

void Robot::ring_receive(vector<float> det) {

    double lidar_depth = std::max((double)det[0], (double)0.2);
    double angle = det[1];

    turn_in_place = false;
    double temp = last_heading + angle;
    heading = (angle > 0) ? last_heading + 60 : last_heading - 60;
    while(abs(temp - imu_val) > 1) delay(5);
    turn_coefficient = 3;
    heading = temp;
    conveyor = -127;
    double coefficient = lidar_depth * meters_to_inches * inches_to_encoder;
    double angle_threshold = 1;

    new_y = y - coefficient * cos(heading / 180 * pi);
    new_x = x + coefficient * sin(heading / 180 * pi);
    while (abs(new_y - y) > 100 || abs(new_x - x) > 100) delay(5);
    delay(500);

    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#");
    turn_in_place = true;
    turn_coefficient = 1;

}

void Robot::receive_fps(nlohmann::json msg){
    if (!started) return;
    double seconds_per_frame = std::stod(msg.dump());
    lcd::print(7, "Seconds per frame: %f", seconds_per_frame);
    last_heading = imu_val;
    if (turn_in_place){
            heading = imu_val + 30;
    }
    if (chasing_mogo) failed_update += 1;
    last_x_gps = (double)cur_x_gps;
    last_y_gps = (double)cur_y_gps;
    last_phi_gps = (double)cur_heading_gps;
}


void Robot::drive(void *ptr) {
    bool flicker_on = false;
    while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool angler_forward = master.get_digital(DIGITAL_L1);
        bool angler_backward = master.get_digital(DIGITAL_L2);

        bool angler_start_thread = master.get_digital(DIGITAL_DOWN);

        bool angler_piston_open = master.get_digital(DIGITAL_A);
        bool angler_piston_close = master.get_digital(DIGITAL_B);

        bool lift_piston_open = master.get_digital(DIGITAL_X);
        bool lift_piston_close = master.get_digital(DIGITAL_Y);

        bool conveyor_forward = master.get_digital(DIGITAL_R1);
        bool conveyor_backward = master.get_digital(DIGITAL_R2);

        if(master.get_digital(DIGITAL_UP)) {
            if(flicker_on) flicker_on = false;
            else flicker_on = true;
        }

        bool lift_up = master.get_digital(DIGITAL_LEFT);
        bool lift_down = master.get_digital(DIGITAL_RIGHT);


        if (angler_backward) angler = 40;
        else if (angler_forward) angler = -40;
        else angler = 0;

        if (angler_start_thread  && !task_exists("ANGLER")) start_task("ANGLER", Robot::depth_angler);

        if (flicker_on) flicker = 127;

        if (angler_piston_open) angler_piston.set_value(true);
        else if (angler_piston_close) angler_piston.set_value(false);

        if (lift_piston_open) lift_piston.set_value(true);
        else if (lift_piston_close) lift_piston.set_value(false);

        if (conveyor_forward) conveyor = 127;
        else if (conveyor_backward) conveyor = -127;
        else conveyor = 0;


        if (lift_up) lift = 127;
        else if (lift_down) lift = -127;
        else lift = 0;

        mecanum(power, strafe, turn);
        delay(5);
    }
}


void Robot::check_depth(void *ptr){

    double depth_threshold = 10;
    std::deque<double> depth_vals;
    double depth_average = 0;

    do {
        if ((int)depth_vals.size() == 10) depth_vals.pop_front();
        depth_vals.push_back(mogo_dist.get());
        double sum = 0;
        for (int i = 0; i < depth_vals.size(); i++) sum += depth_vals[i];
        depth_average = sum / 10;
        delay(5);

        if (failed_update > 1){
            new_y = y + 150 * cos(imu_val / 180 * pi);
            new_x = x - 150 * sin(imu_val / 180 * pi);
        }
    } while (!(abs(depth_average - mogo_dist.get()) < 1 && (mogo_dist.get() > 0 && mogo_dist.get() < 30)));

    delay(350);

    new_x = (float)x;
    new_y = (float)y;

    flicker = 0;
    chasing_mogo = false;
    angler_piston.set_value(true);
    delay(250);
    start_task("ANGLER", Robot::depth_angler);
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#");
    mode = "ring";
    turn_in_place = true;
    kill_task("DEPTH");
    kill_task("MOVETO");
}


void Robot::depth_angler(void *ptr){
    std::deque<double> depth_vals;

    int angler_pot_threshold = 270;
    int depth_threshold = 47;
    int depth_coefficient = 6;
    int cap = 20;
    bool reached = false;
    while (abs(angler_dist.get() - depth_threshold) > cap){
        angler = 127;
    }
    while (true){

        if ((int)depth_vals.size() == 100) depth_vals.pop_front();
        depth_vals.push_back(ring_ultrasonic.get_value());
        double sum = 0;
        for (int i = 0; i < depth_vals.size(); i++) sum += depth_vals[i];
        double depth_average = sum / 100;

        if (abs(angler_dist.get() - depth_threshold) <= cap){
            angler = depth_coefficient * (angler_dist.get() - depth_threshold);
        }
        if (depth_average < 130 && depth_vals.size() == 100){
            delay(250);
            while (angler_pot.get_value() < 2150){
                angler = -127;
            }
            angler = 0;
            angler_piston.set_value(false);
            break;
        }
        delay(5);
    }

    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#stop#true#@#");
    conveyor = 0;
    stop = true;
    new_y = (float)y;
    new_x = (float)x;
    heading = (float)imu_val;
    kill_task("ANGLER");
}


void Robot::imu_clamp(void *ptr){
    double offset = 0;
    while (true){
        double rotation = IMU.get_rotation();
        if (abs(imu_val - rotation) < 10) offset = 0;
        else if (rotation > imu_val) offset = -360;
        else if (rotation < imu_val) offset = 360;

        imu_val = rotation + offset;
        delay(5);
    }
}


void Robot::fps(void *ptr) {
    double last_x = 0;
    double last_y = 0;
    double last_phi = 0;
    double turn_offset_x = 0;
    double turn_offset_y = 0;
    while (true) {
        double cur_phi = imu_val / 180 * pi;
        double dphi = cur_phi - last_phi;

        double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
        double cur_turn_offset_y = 360 * (offset_middle * dphi) / wheel_circumference;

        turn_offset_x = (float)turn_offset_x + cur_turn_offset_x;
        turn_offset_y = (float)turn_offset_y + cur_turn_offset_y;

        double cur_y = (RE.get_value() - LE.get_value()) / 2;
        double cur_x = BE.get_value() + turn_offset_x;

        double dy = cur_y - last_y;
        double dx = cur_x - last_x;

        double global_dy = dy * std::cos(cur_phi) + dx * std::sin(cur_phi);
        double global_dx = dx * std::cos(cur_phi) - dy * std::sin(cur_phi);

        y = (float)y + global_dy;
        x = (float)x + global_dx;

        last_y = cur_y;
        last_x = cur_x;
        last_phi = cur_phi;

        delay(5);
    }
}

//must be run before using any cur or last gps variables
void Robot::gps_fps(void *ptr){
    while (true){
        last_x_gps = (double)cur_x_gps;
        last_y_gps = (double)cur_y_gps;
        pros::c::gps_status_s cur_status = gps.get_status();
        cur_x_gps = cur_status.x;
        cur_y_gps = cur_status.y;
        gps.get_heading() <= 180 ? cur_heading_gps = 180-gps.get_heading() : cur_heading_gps = 540-gps.get_heading();
        lcd::print(1, "Y: %f - X: %f", (float)(cur_y_gps), (float)(cur_x_gps));
        lcd::print(2, "Heading: %f", (float)cur_heading_gps);
        delay(20);
    }
}

/* new_y, new_x, and new_heading_gps are all in absolute field terms
 * gps values are in meters
 * gps values are based on gps camera position not center of the bot
 * (0,0) in the center of the field in meters
 * 0 degrees is facing north (red side on left, blue side on right)
 */
void Robot::move_to_gps(void *ptr) {
    while (true)
    {

        double phi = cur_heading_gps * pi / 180;
        double gps_error;
        double cur_heading_gps2 = cur_heading_gps-360;

        // Lets bot chose shortest path rather than going clockwise/counterclockwise when turning.
        lcd::print(2, "first: %f", (float)std::abs(new_heading_gps-cur_heading_gps));
        lcd::print(3, "second: %f", (float)std::abs(new_heading_gps-cur_heading_gps2));
        if(std::abs(new_heading_gps-cur_heading_gps)<std::abs(new_heading_gps-cur_heading_gps2)){
            gps_error = new_heading_gps - cur_heading_gps;
        }
        else{
            gps_error = new_heading_gps - cur_heading_gps2;
        }
        double y_error = (new_y_gps - cur_y_gps) * meters_to_inches * inches_to_encoder;
        double x_error = (new_x_gps - cur_x_gps) * meters_to_inches * inches_to_encoder;

        double power = power_PD.get_value(y_error * std::cos(phi) - x_error * std::sin(phi));
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) + y_error * std::sin(phi));
        double turn = turn_PD.get_value(gps_error);


        //lcd::print(4, "%f, %f, %f", gps_error, turn, power);

        //Lowers speed of turning when close to ideal heading.
        if(std::abs(power)<=15){
            if(std::abs(power)<=5){
                new_heading_gps = (float)cur_heading_gps;
                turn = 0;
                power = 0;
            }
            turn *= 0.01;
            power *= 0.01;
        }
        mecanum(power, strafe, turn, 127);

        delay(5);
    }
}


void Robot::move_to(void *ptr)
{
    while (true)
    {
        double phi = imu_val * pi / 180;

        double imu_error = -(imu_val - heading);
        double y_error = new_y - y;
        double x_error = -(new_x - x);

        double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
        double turn = turn_PD.get_value(imu_error) * turn_coefficient;

        mecanum(power, strafe, turn, 127);

        delay(5);
    }
}



//threshold can and should be adjusted if return value is inacurate
void Robot::is_moving_gps(int power, int strafe, int max_power, int this_delay) {
    while(true)
    {
        double last_x_gps_slow = cur_x_gps;
        double last_y_gps_slow = cur_y_gps;
        delay(200);
        double cur_x_gps_slow = cur_x_gps;
        double cur_y_gps_slow = cur_y_gps;
        
        double raw_speed = abs((double)power) + abs((double)strafe);
        double theoretical_speed = ((raw_speed > max_power) ? max_power : raw_speed) / 127; //this could maybe be multiplied by some constant, but max speed is pretty close to 1 anyways
        double actual_speed = sqrt(pow(abs(last_x_gps_slow - cur_x_gps_slow), 2) + pow(abs(last_y_gps_slow - cur_y_gps_slow), 2)) * (double)(1000/200); //distance formula
        double threshold = 0.3;
        if (actual_speed > theoretical_speed * threshold) is_moving = true;
        else is_moving = false;
    }
}

void Robot::is_moving_print(void *ptr) {
    while(true) {
        lcd::print(3, "MOVING?: %s", is_moving ? "yes" : "no");
    }
}


void Robot::controller_print(void *ptr){
    while (true){
        master.print(1, 0, "lift pot %d", lift_pot.get_value());
        delay(100);
    }
}


void Robot::display(void *ptr){
    while (true){
        // lcd::print(1, "X: %d, Y: %d, IMU: %d", (int)x, (int)y, (int)imu_val);
        // lcd::print(2, "nX: %d, nY: %d", (int)new_x, (int)new_y);
        lcd::print(3, "%d %d", angler_dist.get(), ring_ultrasonic.get_value());
        delay(5);
    }
}


void Robot::start_task(std::string name, void (*func)(void *)) {
    if (!task_exists(name)) {
        tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
    }
}

bool Robot::task_exists(std::string name) {
    return tasks.find(name) != tasks.end();
}

void Robot::kill_task(std::string name) {
    if (task_exists(name)) {
        tasks.erase(name);
    }
}

void Robot::mecanum(int power, int strafe, int turn, int max_power) {

    int powers[] {
        power + strafe + turn,
        power - strafe - turn,
        power - strafe + turn,
        power + strafe - turn
    };

    int max = *max_element(powers, powers + 4);
    int min = abs(*min_element(powers, powers + 4));

    double true_max = double(std::max(max, min));
    double scalar = (true_max > max_power) ? max_power / true_max : 1;

    FLT = powers[0] * scalar;
    FRT = powers[1] * scalar;
    BLT = powers[2] * scalar;
    BRT = powers[3] * scalar;
    FLB = powers[0] * scalar;
    FRB = powers[1] * scalar;
    BLB = powers[2] * scalar;
    BRB = powers[3] * scalar;
}

void Robot::test(void *ptr) {
    new_y_gps = 1;
    new_x_gps = 0;
    
}
