   
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
ADIDigitalOut Robot::angler_piston(1);
Gps Robot::gps(5, 1.2192, -1.2192, 180, 0, .4064);
Imu Robot::IMU(12);
Distance Robot::angler_dist(21);
Distance Robot::mogo_dist(15);

std::atomic<double> Robot::y = 0;
std::atomic<double> Robot::x = 0;
std::atomic<double> Robot::new_x = 0;
std::atomic<double> Robot::new_y = 0;
std::atomic<double> Robot::heading = 0;
std::atomic<double> Robot::imu_val = 0;
std::atomic<bool> Robot::chasing_mogo = false;
std::atomic<double> Robot::turn_coefficient = 1;

double pi = 3.141592653589793238;
int counter = 0;
int counter2 = 0;
double Robot::offset_back = 5.25;
double Robot::offset_middle = 7.625;
double Robot::wheel_circumference = 2.75 * pi;

std::map<std::string, std::unique_ptr<pros::Task>> Robot::tasks;

const double inches_to_encoder = 41.669;
const double meters_to_inches = 39.3701;

void Robot::receive_mogo(nlohmann::json msg) {

    double angle_threshold = 1;

    if (!task_exists("DEPTH")) start_task("DEPTH", Robot::check_depth);

    string msgS = msg.dump();
    std::size_t found = msgS.find(",");

    double lidar_depth = std::stod(msgS.substr(1, found - 1));
    double angle = std::stod(msgS.substr(found + 1, msgS.size() - found - 1));

    if (angle != 0) {
        chasing_mogo = true;
        //flicker = 127;
    }

    double coefficient = lidar_depth * meters_to_inches * inches_to_encoder;

    if (abs(angle) > angle_threshold) coefficient = 250;
    else if (abs(angle) < angle_threshold && chasing_mogo == true) coefficient = mogo_dist.get() + 100;

    heading = imu_val + angle;
    new_y = y + coefficient * cos(heading / 180 * pi);
    new_x = x - coefficient * sin(heading / 180 * pi);

    lcd::print(3, "X: %d Y: %d L: %d", (int)new_y, (int)new_x, (int)lidar_depth);
    lcd::print(4, "Heading: %d Angle: %d", (int)heading, (int)angle);
    delay(5);
}

void Robot::receive_ring(nlohmann::json msg) {
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#stop#");
    conveyor = -127;
    string msgS = msg.dump();
    std::size_t found = msgS.find(",");

    double lidar_depth = std::stod(msgS.substr(1, found - 1));
    double angle = std::stod(msgS.substr(found + 1, msgS.size() - found - 1));
    double coefficient = lidar_depth * meters_to_inches * inches_to_encoder + 100;
    double angle_threshold = 1;
    double target_heading = imu_val + angle;
    heading = target_heading;
    turn_coefficient = 2;
    while (abs(imu_val - target_heading) > 3){
        // lcd::print(1, "imu: %d, targ: %d", (int)imu_val, (int)target_heading);
        lcd::print(2, "loss: %d", (int)abs(imu_val- target_heading));
        delay(5);
    }
    lcd::print(2, "out");
    new_y = y - coefficient * cos(heading / 180 * pi);
    new_x = x + coefficient * sin(heading / 180 * pi);
    while (abs(new_y - y) > 30 or abs(new_x - x) > 30){
        delay(5);
    }
    conveyor = 0;
    delay(500);
    lcd::print(3, "X: %d Y: %d", (int)y, (int)x);
    lcd::print(3, "nX: %d nY: %d L: %d", (int)new_y, (int)new_x, (int)lidar_depth);
    lcd::print(4, "Heading: %d Angle: %d", (int)heading, (int)angle);
    turn_coefficient = 1;
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#");
}



void Robot::receive_fps(nlohmann::json msg){
    lcd::print(7, "Seconds per frame: %s", msg.dump());
    delay(5);
}


void Robot::drive(void *ptr) {
    while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool angler_forward = master.get_digital(DIGITAL_L1);
        bool angler_backward = master.get_digital(DIGITAL_L2);

        bool angler_start_thread = master.get_digital(DIGITAL_X) || master.get_digital(DIGITAL_DOWN);

        bool angler_piston_open = master.get_digital(DIGITAL_A);
        bool angler_piston_close = master.get_digital(DIGITAL_B);

        bool conveyor_forward = master.get_digital(DIGITAL_R1);
        bool conveyor_backward = master.get_digital(DIGITAL_R2);

        bool flicker_on = master.get_digital(DIGITAL_UP);

        bool lift_up = master.get_digital(DIGITAL_LEFT);
        bool lift_down = master.get_digital(DIGITAL_RIGHT);


        if (angler_backward) angler = 40;
        else if (angler_forward) angler = -40;
        else angler = 0;

        if (angler_start_thread  && !task_exists("ANGLER")) start_task("ANGLER", Robot::depth_angler);

        if (angler_piston_open) angler_piston.set_value(true);
        else if (angler_piston_close) angler_piston.set_value(false);

        // if (conveyor_forward) conveyor = 100;
        // else if (conveyor_backward) conveyor = -100;
        // else conveyor = 0;

        if (flicker_on) flicker = 127;
        else flicker = 0;

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

    while(true){

        if ((int)depth_vals.size() == 10) depth_vals.pop_front();
        depth_vals.push_back(mogo_dist.get());
        double sum = 0;
        for (int i = 0; i < depth_vals.size(); i++) sum += depth_vals[i];
        double depth_average = sum / 10;

        if (abs(depth_average - mogo_dist.get()) < 1 && mogo_dist.get() > 0 && mogo_dist.get() < 50){
            new_x = (float)x;
            new_y = (float)y;
            lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#stop#");

            // flicker = 0;
            // chasing_mogo = false;
            // angler_piston.set_value(true);
            // delay(250);
            // start_task("ANGLER", Robot::depth_angler);

            kill_task("DEPTH");
        }
        delay(5);
    }
}


void Robot::depth_angler(void *ptr){
    int angler_pot_threshold = 270;
    int depth_threshold = 42;
    int depth_coefficient = 3;
    while (true){

        if(master.get_digital(DIGITAL_Y)) break;

        if(master.get_digital(DIGITAL_DOWN)) {
            while (angler_pot.get_value() < 2415){
                angler = -127;
            }
            break;
        }

        if (abs(angler_pot.get_value() - angler_pot_threshold) > 50){
            if (angler_pot.get_value() > angler_pot_threshold) angler = 127;
            else angler = -127;
        }
        else {
            double distance = angler_dist.get();
            if (abs(distance - depth_threshold) < 10) angler = depth_coefficient * (angler_dist.get() - depth_threshold);
            else angler = 0;
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
        double cur_phi = IMU.get_rotation() / 180 * pi;
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

        //lcd::print(1,"Y: %d X: %d IMU: %f", (int)y, (int)x, IMU.get_rotation());
        //lcd::print(2,"angler_pot %d - Dist. %d", angler_pot.get_value(), angler_dist.get());

        delay(5);
    }
}



void Robot::gps_fps(void *ptr){
    while (true){
        lcd::print(5, "Y: %f - X: %f", (float)gps.get_status().y, (float)gps.get_status().x);
        lcd::print(6, "Heading: %f", (float)gps.get_heading());
        delay(5);
    }
}


void Robot::move_to(void *ptr)
{
    while (true)
    {
        double phi = (IMU.get_rotation()) * pi / 180;

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


void Robot::controller_print(void *ptr){
    while (true){
        master.print(1, 0, "lift pot %d", lift_pot.get_value());
        delay(100);
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