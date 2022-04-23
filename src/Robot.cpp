
#include "main.h"
#include "Robot.h"
#include "system/Data.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "PD.h"
#include <map>
#include <cmath>
#include <string>
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
Gps Robot::gps(5);
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

std::atomic<int> Robot::stagnant = 0;

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
std::atomic<bool> resetting = false;
std::atomic<int> move_to_mode = 0; //0 = fps, 1 = gps
std::atomic<int> move_to_count = 0;
std::atomic<int> move_to_gps_count = 0;
int mogo_count = 0;

GridMapper* gridMapper = new GridMapper();


std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;


void Robot::receive_data(nlohmann::json msg)
{
    double position_temp[] = {gps.get_status().x*meters_to_inches + 72, gps.get_status().y*meters_to_inches + 72, pi/4};
    std::map<std::string, std::vector<double*>> objects;
    string names[] = {"ring", "mogo"};
    if (stop) return;
    started = true;
    stagnant = 0;
    resetting = true;
    vector<vector<float>> pred = Data::get_pred(msg);
    
    // for (vector<double> det : objects) {
    //     double location[] = {det[0] * meters_to_inches, det[1]*-1/180*pi};
    //     objects[names[det[2]]].push_back(location);
    // }
    
    // double position_temp[] = {gps.get_status().x*meters_to_inches + 72, gps.get_status().y*meters_to_inches + 72, pi/4};
    // gridMapper->map(position_temp, objects); 
    
    
    if (mode.compare("mogo") == 0){
        vector<vector<float>> mogos = Data::pred_id(pred, 0);
        for (vector<float> det : mogos){
            if (Data::invalid_det(det, cur_x_gps, cur_y_gps, cur_heading_gps)) {
                continue;
            }
            mogo_count += 1;
            if (mogo_count > 1 || chasing_mogo) mogo_receive(det);
            return;
        }
        mogo_count = 0;
    }
    if (mode.compare("ring") == 0){
        vector<vector<float>> rings = Data::pred_id(pred, 1);
        for (vector<float> det : rings){
            det[0] += 0.2;
            if (Data::invalid_det(det, cur_x_gps, cur_y_gps, cur_heading_gps)) {
                continue;
            }
            ring_receive(det);
            break;
        }
        lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");
    }
}


void Robot::dummy(nlohmann::json msg){
    vector<vector<float>> pred = Data::get_pred(msg);
    int valid = 0;
    int invalid = 0;
    vector<vector<float>> rings = Data::pred_id(pred, 1);
    for (vector<float> det : rings){
        det[0] += 0.2;
        if (Data::invalid_det(det, last_x_gps, last_y_gps, last_phi_gps)) {
            invalid += 1;
        }
        else {
            valid += 1;
        }
    }
    lcd::print(6, "VALID %d INVALID %d", valid, invalid);
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");

}



void Robot::mogo_receive(vector<float> det)
{
  //copy and pasted, changed to work with the attributes given by receive_data
  failed_update = 0;
  move_to_mode = 0;
  turn_in_place = false;
  chasing_mogo = true;
  flicker = 127;

  double angle_threshold = 1;
  turn_coefficient = 3;

  if (!task_exists("DEPTH")) start_task("DEPTH", Robot::check_depth);


  double lidar_depth = std::max((double)det[0], (double)0.2);
  double angle = det[1];

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
    move_to_mode = 0;
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


void Robot::reset(void *ptr) {
    stagnant = 0;
    while (true) {
        if (stagnant > 10 && resetting){            
            stagnant = 0;
            conveyor = 0;
            move_to_mode = 1;
            new_y_gps = cur_y_gps / 2;
            new_x_gps = cur_x_gps / 2;
            while (stagnant < 5){
                lcd::print(5, "resetting");
                delay(5);
            }
            lcd::print(5, "reset");
            stay();
            lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");
            move_to_mode = 0;
            stagnant = 0;
        }
        delay(5);
    }
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

    stay();

    flicker = 0;
    chasing_mogo = false;
    angler_piston.set_value(true);
    delay(250);
    start_task("ANGLER", Robot::depth_angler);
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
    mode = "ring";
    turn_in_place = true;
    resetting = false;
    kill_task("DEPTH");
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

            resetting = false;
            conveyor = 0;
            stop = true;

            move_to_mode = 1;
            new_y_gps = 0;
            new_x_gps = 0;
            new_heading_gps = 135;
            
            stagnant = 0;
            while (stagnant < 5) {
                delay(5);
                angler = depth_coefficient * (angler_dist.get() - depth_threshold);
            }

            new_y_gps = -1.2;
            new_x_gps = -1.2;

            stagnant = 0;
            while (stagnant < 5) {
                delay(5);
                angler = depth_coefficient * (angler_dist.get() - depth_threshold);
            }
            stay();

            while (angler_pot.get_value() < 2300) angler = -(2300 - angler_pot.get_value());

            angler = 0;
            angler_piston.set_value(false);
            break;
        }
        delay(5);
    }
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
        pros::c::gps_status_s cur_status = gps.get_status();
        cur_x_gps = cur_status.x;
        cur_y_gps = cur_status.y;
        cur_heading_gps = gps.get_heading();
        lcd::print(1, "Y: %f - X: %f", (float)(cur_y_gps), (float)(cur_x_gps));
        lcd::print(2, "Heading: %f", (float)cur_heading_gps);
        delay(5);
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
        if (move_to_mode != 1) {
            delay(5);
            continue;
        }
        move_to_gps_count = (int)move_to_gps_count + 1;
        double phi = cur_heading_gps * pi / 180;
        double gps_error;
        double cur_heading_gps2 = cur_heading_gps - 360;

        if(std::abs(new_heading_gps-cur_heading_gps) < std::abs(new_heading_gps-cur_heading_gps2)){
            gps_error = new_heading_gps - cur_heading_gps;
        }
        else{
            gps_error = new_heading_gps - cur_heading_gps2;
        }
        double y_error = -(new_y_gps - cur_y_gps) * meters_to_inches * inches_to_encoder;
        double x_error = -(new_x_gps - cur_x_gps) * meters_to_inches * inches_to_encoder;

        double power = power_PD.get_value(y_error * std::sin(phi) - x_error * std::cos(phi));
        double strafe = strafe_PD.get_value(x_error * std::sin(phi) + y_error * std::cos(phi));
        double turn = turn_PD.get_value(gps_error);

        mecanum(power, strafe, turn, 127);

        delay(5);
    }
}


void Robot::move_to(void *ptr)
{
    while (true)
    {
        if (move_to_mode != 0) {
            delay(5);
            continue;
        }
        move_to_count = (int)move_to_count + 1;
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
void Robot::is_moving_gps(void *ptr) {
    double xy_threshold = 20;
    double turn_threshold = 0.5;
    while(true)
    {
        double last_x_slow = (float)x;
        double last_y_slow = (float)y;
        double last_heading_slow = gps.get_heading();
        delay(200);
        double cur_x_slow = (float)x;
        double cur_y_slow = (float)y;
        double cur_heading_slow = gps.get_heading();

        double xy_diff = abs(last_x_slow - cur_x_slow) + abs(last_y_slow - cur_y_slow);
        double turn_diff = cur_heading_slow - last_heading_slow;

        if (xy_diff < xy_threshold && turn_diff < turn_threshold) stagnant = (int)stagnant + 1;
        else stagnant = 0;
    }
}


void Robot::display(void *ptr){
    while (true){
        lcd::print(6, "MOVETO %d MOVETOGPS %d", (int)move_to_count, (int)move_to_gps_count);
        lcd::print(3, "STAGNANT: %d", (int)stagnant);
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

void Robot::stay(){
    new_y = (float)y;
    new_x = (float)x;
    heading = (float)imu_val;

    new_x_gps = (float)cur_x_gps;
    new_y_gps = (float)cur_y_gps;
    new_heading_gps = gps.get_heading();
}

