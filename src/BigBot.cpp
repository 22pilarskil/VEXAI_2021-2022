
#include "main.h"
#include "BigBot.h"
#include "system/Data.h"
#include "system/json.hpp"
#include "system/Serial.h"
#include "system/Data.h"
#include "utils/PD.h"
#include "utils/GridMapper.cpp"
#include <map>
#include <cmath>
#include <string>
#include <atomic>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <deque>
using namespace pros;
using namespace std;

Controller BigBot::master(E_CONTROLLER_MASTER);
PD BigBot::power_PD(0.4, 0, 0);
PD BigBot::strafe_PD(0.4, 0, 0);
PD BigBot::turn_PD(1.0, 0, 0);

Motor BigBot::BRB(1, true);
Motor BigBot::BRT(3);
//for some reason BRB can't be flipped to true, so I've added negatives to BRB and BRT's motor values in mecanum instead

Motor BigBot::BLT(10, true);
Motor BigBot::BLB(9);
Motor BigBot::FLT(19, true);
Motor BigBot::FLB(18);
Motor BigBot::FRB(11, true);
Motor BigBot::FRT(6);
Motor BigBot::angler(20);
Motor BigBot::conveyor(2);

ADIEncoder BigBot::LE({{17, 1, 2}});
ADIEncoder BigBot::RE({{17, 5, 6}});
ADIEncoder BigBot::BE(7, 8);
ADIAnalogIn BigBot::angler_pot(4);
ADIDigitalOut BigBot::angler_piston(2);
ADIUltrasonic BigBot::ring_ultrasonic(5, 6);
Gps BigBot::gps(8);
Imu BigBot::IMU(12);
Distance BigBot::angler_dist(21);
Distance BigBot::mogo_dist(7);

const double inches_to_encoder = 41.669;
const double meters_to_inches = 39.3701;
const double pi = 3.141592653589793238;

std::atomic<double> BigBot::x = 0;
std::atomic<double> BigBot::y = 0;
std::atomic<double> BigBot::heading = 0;
std::atomic<double> BigBot::new_x = 0;
std::atomic<double> BigBot::new_y = 0;
std::atomic<double> BigBot::new_heading = 0;

std::atomic<double> BigBot::x_gps = 0;
std::atomic<double> BigBot::y_gps = 0;
std::atomic<double> BigBot::heading_gps = 0;
std::atomic<double> BigBot::new_x_gps = 0;
std::atomic<double> BigBot::new_y_gps = 0;
std::atomic<double> BigBot::new_heading_gps = 0;
std::atomic<double> BigBot::last_x_gps = 0;
std::atomic<double> BigBot::last_y_gps = 0;
std::atomic<double> BigBot::last_heading_gps = 0;

std::atomic<double> BigBot::drive_temp = 0;


std::atomic<int> BigBot::stagnant = 0;

std::string BigBot::mode = "mogo";
bool BigBot::stop = false;

double BigBot::offset_back = 5.25;
double BigBot::offset_middle = 7.625;
double BigBot::wheel_circumference = 2.75 * pi;



std::atomic<bool> chasing_mogo = false;
std::atomic<double> turn_coefficient = 1;
std::atomic<bool> turn_in_place = true;
double seconds_per_frame = 0.20;
int failed_update = 0;
double last_heading = 0;
bool started = false;
std::atomic<bool> resetting = false;

//Both move_to threads will be running at the same time, move_to_mode tells the brain which thread we will actually be using
std::atomic<int> move_to_mode = 0; //0 = fps, 1 = gps

std::atomic<int> move_to_count = 0;
std::atomic<int> move_to_gps_count = 0;
int mogo_count = 0;
int corner = 0;
int angler_finish = 0; 
int angler_speed = 0;
int angler_mode = 0;

GridMapper* bigBotGrid = new GridMapper();


std::map<std::string, std::unique_ptr<pros::Task>> BigBot::tasks;


void BigBot::receive_data(nlohmann::json msg)
{
    double position_temp[] = {gps.get_status().x*meters_to_inches + 72, gps.get_status().y*meters_to_inches + 72, pi/4};
    std::map<std::string, std::vector<double*>> objects;
    string names[] = {"ring", "mogo"};
    if (stop) return;

    started = true;
    stagnant = 0;
    vector<vector<float>> pred = Data::get_pred(msg);
    if(pred.empty())return;

    if (mode.compare("mogo") == 0){
        vector<vector<float>> mogos = Data::pred_id(pred, 0);
        for (vector<float> det : mogos){
            det[0] += 0.4;
            if (Data::invalid_det(det, last_x_gps, last_y_gps, 360-last_heading_gps)) {
                continue;
            }
            det[0] -= 0.4;
            //This is mainly because our model is bad and occaisionally we have isolated mogo detections on empty areas, this forces the bot to detect a mogo at least twice in a row before going after it
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

            if(Data::invalid_det(det, last_x_gps, last_y_gps, fmod(540-last_heading_gps, 360))) {//opposite camera so have to do different stuff to make it unit circle
                continue;
            }
            ring_receive(det);
            break;
        }
        lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");
    }
}

void BigBot::motor_temperature(void *ptr)
{
    while(true)
    {
        drive_temp = 0;
        drive_temp = drive_temp + BigBot::BRB.get_temperature();
        drive_temp = drive_temp + BigBot::BRT.get_temperature();
        drive_temp = drive_temp + BigBot::BLB.get_temperature();
        drive_temp = drive_temp + BigBot::BLT.get_temperature();
        drive_temp = drive_temp + BigBot::FRB.get_temperature();
        drive_temp = drive_temp + BigBot::FRT.get_temperature();
        drive_temp = drive_temp + BigBot::FLB.get_temperature();
        drive_temp = drive_temp + BigBot::FLT.get_temperature();
        drive_temp = drive_temp / 8;
        delay(5);
    }
}

void BigBot::dummy(nlohmann::json msg){
    vector<vector<float>> pred = Data::get_pred(msg);
    int valid_rings = 0;
    int invalid_rings = 0;
    int valid_mogos = 0;
    int invalid_mogos = 0;
    vector<vector<float>> rings = Data::pred_id(pred, 1);
    for (vector<float> det : rings){
        det[0] += 0.2;
        if (Data::invalid_det(det, last_x_gps, last_y_gps, fmod(540-last_heading_gps, 360))) {
            invalid_rings += 1;
        }
        else {
            valid_rings += 1;
        }
    }
    vector<vector<float>> mogos = Data::pred_id(pred, 0);
    for (vector<float> det : mogos){
        det[0] += 0.4;
        if (Data::invalid_det(det, last_x_gps, last_y_gps, 360-last_heading_gps)) {
            invalid_mogos += 1;
        }
        else {
            valid_mogos += 1;
        }
    }
    //lcd::print(6, "VR %d IR %d VM %d IM %d", valid_rings, invalid_rings, valid_mogos, invalid_mogos);
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");

}


void BigBot::mogo_receive(vector<float> det)
{
  failed_update = 0;
  move_to_mode = 0;
  turn_in_place = false;
  chasing_mogo = true;

  double angle_threshold = 1;
  turn_coefficient = 3;

  if (!task_exists("DEPTH")) start_task("DEPTH", BigBot::check_depth);


  double lidar_depth = std::max((double)det[0], (double)0.2);
  double angle = det[1];

  double coefficient;

  if (abs(angle) > angle_threshold) coefficient = 150 * lidar_depth * (0.20 / seconds_per_frame);
  else if (abs(angle) < angle_threshold && chasing_mogo == true) coefficient = 300;

  new_heading = heading + angle;
  new_y = y + coefficient * cos(new_heading / 180 * pi);
  new_x = x - coefficient * sin(new_heading / 180 * pi);

  delay(5);
  turn_coefficient = 1;
}

void BigBot::ring_receive(vector<float> det) {

    resetting = true;
    double lidar_depth = std::max((double)det[0], (double)0.2);
    double angle = det[1];
    move_to_mode = 0;
    turn_in_place = false;
    double temp = last_heading + angle;
    new_heading = (angle > 0) ? last_heading + 60 : last_heading - 60;
    while(abs(temp - heading) > 1 && stagnant < 10) delay(5);
    turn_coefficient = 3;
    new_heading = temp;
    conveyor = -127;
    double coefficient = lidar_depth * meters_to_inches * inches_to_encoder;
    double angle_threshold = 1;

    new_y = y - coefficient * cos(new_heading / 180 * pi);
    new_x = x + coefficient * sin(new_heading / 180 * pi);
    while ((abs(new_y - y) > 100 || abs(new_x - x) > 100) && stagnant < 10) delay(5);
    //checks if bot is too close to the wall and balance on the sides and goes back to the middle

    move_to_mode = 1;
    delay(500);

    if (abs(x_gps) > 0.65 || abs(y_gps) > 1.15) {
        new_x_gps = new_x_gps / 2;
        new_y_gps = new_y_gps / 2;
        while (!(abs(new_x_gps - x_gps) < 0.1 && abs(new_y_gps - y_gps) < 0.1)){
            delay(5);
        }
    }
    stay();

    move_to_mode = 0;

    turn_in_place = true;
    turn_coefficient = 1;
    resetting = false;

}

void BigBot::receive_fps(nlohmann::json msg){
    double seconds_per_frame = std::stod(msg.dump());
    //, "Seconds per frame: %f", seconds_per_frame);
    last_heading = heading;
    if (turn_in_place){
        new_heading = heading + 30;
    }
    if (chasing_mogo) {failed_update += 1;}
    last_x_gps = (double)x_gps;
    last_y_gps = (double)y_gps;
    last_heading_gps = (double)heading_gps;
}
void BigBot::reposition(void *ptr)
{
    double turn_degree = 0;
    double last_imu_angle = 0;
    
    while(true){
        if(!turn_in_place){
            last_imu_angle = heading;
            turn_degree = 0;
            delay(5);
        }
        else if(turn_in_place){
            turn_degree += abs(heading - last_imu_angle);
            last_imu_angle = heading;
            if(turn_degree > 360){
                turn_in_place = false;
                move_to_mode = 1;
                new_y_gps = x_gps/2;
                new_x_gps = x_gps/2;
                while (!(abs(new_x_gps - x_gps) < 0.1 && abs(new_y_gps - y_gps) < 0.1)){
                    delay(5);
                }
                stay();
                turn_in_place = true;
                conveyor = 127;
                move_to_mode = 0;
                stagnant = 0;
                turn_degree = 0;
                last_imu_angle = heading;
                continue;
            }
            delay(5);
        }
    }

}

/* Uses the stagnant variable from BigBot::is_moving to tell whether the BigBot hit an obstacle (i.e. ran into a balance)
If the BigBot has run into an obstacle, move the BigBot closer to the center of the field by halving both coordinates so as
to bring the bot closer to (0, 0). resetting variable is used to keep track of whether or not we actually want to be using
this thread. For example, if we are depositing a mogo in a corner or some other action where the bot will not be moving
temporarily, we would set resetting to false so that the if statement is never called */
void BigBot::reset(void *ptr) {
    stagnant = 0;
    lcd::print(6, "STARTED");
    while (true) {
        if (stagnant > 10 && resetting){
            lcd::print(6, "RESETTING");
            delay(50);
            stagnant = 0;
            conveyor = 0;
            move_to_mode = 1;
            stay();
            new_y_gps = y_gps / 2;
            new_x_gps = x_gps / 2;
            while (!(abs(new_x_gps - x_gps) < .1 && abs(new_y_gps - y_gps) < .1)){
                delay(5);
            }
            stay();

            /*After a reset, send a continue_ring signal so that if the BigBot was going after rings, it starts looking again. If
            it was going after mogos, nothing happens. Set chasing_mogo to false to tell the BigBot it needs to look for a new target,
            turn_in_place to true so the BigBot starts turning once more, and move_to_mode back to 0 so that our fps move_to can take
            care of our turning in place. Reset stagnant back to 0 so we don't call reset accidentally again. */
            lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue_ring#true#@#");
            chasing_mogo = false;
            turn_in_place = true;
            move_to_mode = 0;
            stagnant = 0;
            lcd::print(6, "stopping");
        }

        delay(5);
    }
}


void BigBot::drive(void *ptr) {
    while (true) {
        int power = master.get_analog(ANALOG_LEFT_Y);
        int strafe = master.get_analog(ANALOG_LEFT_X);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        bool angler_forward = master.get_digital(DIGITAL_L1);
        bool angler_backward = master.get_digital(DIGITAL_L2);

        bool angler_start_thread = master.get_digital(DIGITAL_DOWN);

        bool angler_piston_open = master.get_digital(DIGITAL_A);
        bool angler_piston_close = master.get_digital(DIGITAL_B);

        bool conveyor_forward = master.get_digital(DIGITAL_R1);
        bool conveyor_backward = master.get_digital(DIGITAL_R2);

        if (angler_backward) angler = -40;
        else if (angler_forward) angler = 40;
        else angler = 0;

        if (angler_start_thread  && !task_exists("ANGLER")) start_task("ANGLER", BigBot::depth_angler);
        if (angler_piston_open) angler_piston.set_value(true);
        else if (angler_piston_close) angler_piston.set_value(false);

        if (conveyor_forward) conveyor = 127;
        else if (conveyor_backward) conveyor = -127;
        else conveyor = 0;


        mecanum(power, strafe, turn);
        delay(5);
    }
}


void BigBot::check_depth(void *ptr){

    double depth_threshold = 10;
    std::deque<double> depth_vals;
    double depth_average = 0;

    do {
        if ((int)depth_vals.size() == 50) depth_vals.pop_front();
        depth_vals.push_back(mogo_dist.get());
        double sum = 0;
        for (int i = 0; i < depth_vals.size(); i++) sum += depth_vals[i];
        depth_average = sum / 50;
        delay(5);

        if (failed_update > 1){
            new_y = y + 150 * cos(heading / 180 * pi);
            new_x = x - 150 * sin(heading / 180 * pi);
        }
    } while (!(abs(depth_average - mogo_dist.get()) < 1 && (mogo_dist.get() > 0 && mogo_dist.get() < 30)));

    delay(100);

    stay();

    chasing_mogo = false;
    angler_piston.set_value(true);

    delay(250);
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#@#");
    start_task("ANGLER", BigBot::depth_angler);
    mode = "ring";
    turn_in_place = true;
    kill_task("DEPTH");
}


void BigBot::depth_angler(void *ptr){
    std::deque<double> depth_vals;

    int depth_threshold = 45;
    int depth_coefficient = 12;
    int cap = 1;
    bool reached = false;
    while (abs(angler_dist.get() - depth_threshold) > cap){
        angler = -127;
    }
    angler_finish = angler_pot.get_value();
    while (true){

        if ((int)depth_vals.size() == 100) depth_vals.pop_front();
        depth_vals.push_back(ring_ultrasonic.get_value());
        double sum = 0;
        for (int i = 0; i < depth_vals.size(); i++) sum += depth_vals[i];
        double depth_average = sum / 100;


        if (abs(angler_pot.get_value() - angler_finish) > 300){
            angler_speed = .25 * (angler_pot.get_value() - angler_finish);
            angler = angler_speed;
            angler_mode = 0;
        }
        else {
            angler_speed = -1 * depth_coefficient * (angler_dist.get() - depth_threshold);
            angler = angler_speed;
            angler_mode = 1;
        }

        //When a mogo has been filled
        if (false){//(depth_average < 200 && depth_vals.size() == 100){

            stop = true;
            //switch to move_to_gps, move to the center of the field
            move_to_mode = 1;
            new_y_gps = 0;
            new_x_gps = 0;
            if(corner == 0) new_heading_gps = 135;
            else if(corner == 1) new_heading_gps = 315;

            //make sure bot has stopped moving (aka reached its target)
            stagnant = 0;
            while (!(abs(new_x_gps - x_gps) < .1 && abs(new_y_gps - y_gps) < .1 && abs(new_heading_gps - gps.get_heading()) < 3)){
                delay(5);
                angler = .25 * (angler_pot.get_value() - angler_finish);
            }

            //move to corner of the field to deposit mogo
            if(corner == 0){
                new_y_gps = -1.3;
                new_x_gps = -1.2;
                corner = 1;
            }
            else if(corner == 1){
                new_y_gps = 1.3;
                new_x_gps = 1.2;
                corner = 0;
            }


            //make sure bot has stopped moving (aka reached its target)
            stagnant = 0;
            while (!(abs(new_x_gps - x_gps) < .1 && abs(new_y_gps - y_gps) < .1 && abs(new_heading_gps - gps.get_heading()) < 3)){
                delay(5);
                angler = .25 * (angler_pot.get_value() - angler_finish);
            }
            stay();
            lcd::print(7, "REASCHED");

            //release mogo
            while (angler_pot.get_value() > 1250) angler =  - (1250 - angler_pot.get_value());

            angler = 0;
            angler_piston.set_value(false);
            delay(200);
            new_y_gps = 0;
            new_x_gps = 0;
            mode = "mogo";
            lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_back#@#");
            while (!(abs(new_x_gps - x_gps) < .1 && abs(new_y_gps - y_gps) < .1 && abs(new_heading_gps - gps.get_heading()) < 3)){
                delay(5);
            }
            stay();
            stop = false;
            move_to_mode = 0;
            turn_in_place = true;
            break;
        }
        delay(5);
    }
    kill_task("ANGLER");
}


void BigBot::imu_clamp(void *ptr){
    double offset = 0;
    while (true){
        double rotation = IMU.get_rotation();
        if (abs(heading - rotation) < 10) offset = 0;
        else if (rotation > heading) offset = -360;
        else if (rotation < heading) offset = 360;
        heading = rotation + offset;
        delay(5);
    }
}


void BigBot::fps(void *ptr) {

    double last_x = 0;
    double last_y = 0;
    double last_phi = 0;
    double turn_offset_x = 0;

    while (true) {
        double cur_phi = heading / 180 * pi;
        double dphi = cur_phi - last_phi;

        double cur_turn_offset_x = 360 * (offset_back * dphi) / wheel_circumference;
        turn_offset_x = (float)turn_offset_x + cur_turn_offset_x;

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
void BigBot::gps_fps(void *ptr){
    int i = 0;
    while (true){
        pros::c::gps_status_s cur_status = gps.get_status();
        x_gps = cur_status.x;
        y_gps = cur_status.y;
        heading_gps = gps.get_heading();
        delay(5);
    }
}

/* new_y, new_x, and new_heading_gps are all in absolute field terms
 * gps values are in meters
 * gps values are based on gps camera position not center of the bot
 * (0,0) in the center of the field in meters
 * 0 degrees is facing north (red side on left, blue side on right)
 */
void BigBot::move_to_gps(void *ptr) {
    while (true)
    {
        if (move_to_mode != 1) {
            delay(5);
            continue;
        }
        move_to_gps_count = (int)move_to_gps_count + 1;
        double phi = heading_gps * pi / 180;
        double gps_error;
        double heading_gps2 = heading_gps - 360;

        if(std::abs(new_heading_gps-heading_gps) < std::abs(new_heading_gps-heading_gps2)){
            gps_error = new_heading_gps - heading_gps;
        }
        else{
            gps_error = new_heading_gps - heading_gps2;
        }
        double y_error = -(new_y_gps - y_gps) * meters_to_inches * inches_to_encoder;
        double x_error = -(new_x_gps - x_gps) * meters_to_inches * inches_to_encoder;

        double power = power_PD.get_value(y_error * std::sin(phi) - x_error * std::cos(phi));
        double strafe = strafe_PD.get_value(x_error * std::sin(phi) + y_error * std::cos(phi));
        double turn = turn_PD.get_value(gps_error);
        mecanum(power, strafe, turn, 127);

        delay(5);
    }
}


void BigBot::move_to(void *ptr)
{
    while (true)
    {
        if (move_to_mode != 0) {
            delay(5);
            continue;
        }
        move_to_count = (int)move_to_count + 1;
        double phi = heading * pi / 180;

        double imu_error = -(heading - new_heading);
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
void BigBot::is_moving_gps(void *ptr) {
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


void BigBot::display(void *ptr){
    while (true){

        lcd::print(1, "FPS: X %d Y %d IMU %d", (int)x, (int)y, (int)IMU.get_rotation());
        lcd::print(2, "GPS: X %.2f Y: %.2f HEADING: %.2f", (float)(x_gps), (float)(y_gps), (float)(heading_gps));
        lcd::print(3, "STAGNANT FOR: %d", (int)stagnant);
        lcd::print(4, "TEMP: %f", (float)drive_temp);
        lcd::print(5, "%d", ring_ultrasonic.get_value());
        lcd::print(6, "Dist %d POT %d AM %d", angler_dist.get(), angler_pot.get_value(), angler_speed);
        delay(5);
    }
}


void BigBot::start_task(std::string name, void (*func)(void *)) {
    if (!task_exists(name)) {
        tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(name, std::move(std::make_unique<pros::Task>(func, &x, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, ""))));
    }
}

bool BigBot::task_exists(std::string name) {
    return tasks.find(name) != tasks.end();
}

void BigBot::kill_task(std::string name) {
    if (task_exists(name)) {
        tasks.erase(name);
    }
}

void BigBot::mecanum(int power, int strafe, int turn, int max_power) {

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
    BRT = -powers[3] * scalar;
    FLB = powers[0] * scalar;
    FRB = powers[1] * scalar;
    BLB = powers[2] * scalar;
    BRB = powers[3] * scalar;
}

void BigBot::stay(){
    new_y = (float)y;
    new_x = (float)x;
    new_heading = (float)heading;

    new_x_gps = (float)x_gps;
    new_y_gps = (float)y_gps;
    new_heading_gps = gps.get_heading();
}
