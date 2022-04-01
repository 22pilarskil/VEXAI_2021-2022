
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

double Robot::offset_back = 5.25;
double Robot::offset_middle = 7.625;
double Robot::wheel_circumference = 2.75 * pi;


std::atomic<bool> chasing_mogo = false;
std::atomic<double> turn_coefficient = 1;
std::atomic<bool> turn_in_place = true;
double seconds_per_frame = 0.20;
int failed_update = 0;
double last_heading = 0;

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
  lcd::print(8, "HIHIHIHIHI");
  //data in form rings(det[depth, angle],...)!red()!yellow()!blue()!
  string whole_str = msg.dump();
  vector<string> color_separated;

  string delimiter = "!";
  //data loading
  int pos = 0;
  string token;
  while ((pos = whole_str.find(delimiter)) != std::string::npos) {
    token = whole_str.substr(0, pos);
    color_separated.push_back(token);
    whole_str.erase(0, pos + delimiter.length());
  }


  vector<vector<double>> ring_dets;
  vector<vector<double>> red_mogo_dets;
  vector<vector<double>> yellow_mogo_dets;
  vector<vector<double>> blue_mogo_dets;

  string det_delimiter = "|";
  string attribute_delimiter = ",";

  //ring dets
  pos = 0;

  while ((pos = color_separated[0].find(det_delimiter)) != std::string::npos) {
    token = color_separated[0].substr(0, pos);
    int pos_1 =0;
    string token1;
    vector<double> temp;
    while((pos_1 = token.find(attribute_delimiter)!=std::string::npos))
    {
      token1 = token.substr(0,pos_1);
      temp.push_back(std::stod(token1));
      token.erase(0,pos_1+attribute_delimiter.length());
    }
    ring_dets.push_back(temp);
    color_separated[0].erase(0, pos + delimiter.length());
  }
  //red_mogo_dets
  pos = 0;
  while ((pos = color_separated[1].find(det_delimiter)) != std::string::npos) {
    token = color_separated[1].substr(0, pos);
    int pos_1 =0;
    string token1;
    vector<double> temp;
    while((pos_1 = token.find(attribute_delimiter)!=std::string::npos))
    {
      token1 = token.substr(0,pos_1);
      temp.push_back(std::stod(token1));
      token.erase(0,pos_1+attribute_delimiter.length());
    }
    red_mogo_dets.push_back(temp);
    color_separated[1].erase(0, pos + delimiter.length());
  }

  //yellow_mogo_dets
  pos = 0;
  while ((pos = color_separated[2].find(det_delimiter)) != std::string::npos) {
    token = color_separated[2].substr(0, pos);
    int pos_1 =0;
    string token1;
    vector<double> temp;
    while((pos_1 = token.find(attribute_delimiter)!=std::string::npos))
    {
      token1 = token.substr(0,pos_1);
      temp.push_back(std::stod(token1));
      token.erase(0,pos_1+attribute_delimiter.length());
    }
    yellow_mogo_dets.push_back(temp);
    color_separated[2].erase(0, pos + delimiter.length());
  }
  //blue_mogo_dets
  pos = 0;
  while ((pos = color_separated[2].find(det_delimiter)) != std::string::npos) {
    token = color_separated[2].substr(0, pos);
    int pos_1 =0;
    string token1;
    vector<double> temp;
    while((pos_1 = token.find(attribute_delimiter)!=std::string::npos))
    {
      token1 = token.substr(0,pos_1);
      temp.push_back(std::stod(token1));
      token.erase(0,pos_1+attribute_delimiter.length());
    }
    blue_mogo_dets.push_back(temp);
    color_separated[2].erase(0, pos + delimiter.length());
  }
  /**
    do stuff with these dets
    load stuff into the map

  **/
  ring_receive(ring_dets);

}
void Robot::ring_receive(vector<vector<double>> input)
{
  //changed to work with inputs from receive_data
  turn_in_place = false;
  heading = last_heading;
  turn_coefficient = 3;
  while(abs(heading - imu_val) > 3) delay(5);

  conveyor = -127;

  //find best input
  bool found = false;
  int pos = 0;
  double x = gps.get_status().x;
  double y = gps.get_status().y;
  double gps_heading = gps.get_heading();

  double depth;
  double ang;
  while(!found && pos<input.size())
  {
    lcd::print(10, "SUOISDIHSDIOSHDUISDUIHSDIUHHUISDHUIDHIUDUIHSIUH");
    double lidar_depth = input[pos][0];
    double angle = input[pos][1];

    double sin_f = sin(gps_heading)*lidar_depth+y;
    double cos_f = cos(gps_heading)*lidar_depth+x;

    if(sin_f<=-2.1*12/meters_to_inches || sin_f>=2.1*12/meters_to_inches || cos_f <= -2.1*12/meters_to_inches
    || cos_f >= 2.1*12/meters_to_inches)
    {
      pos++;
    }
    else
    {
      found = true;
      depth = lidar_depth;
      ang = angle;
      break;
    }
  }

  if(found)
  {
    double coefficient = depth * meters_to_inches * inches_to_encoder + 300;
    double angle_threshold = 1;
    double target_heading = imu_val + ang;
    heading = target_heading;
    while (abs(imu_val - target_heading) > 3) delay(5);

    new_y = y - coefficient * cos(heading / 180 * pi);
    new_x = x + coefficient * sin(heading / 180 * pi);
    while (abs(new_y - y) > 100 or abs(new_x - x) > 100) delay(5);

    conveyor = 0;
    delay(500);

    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#");
    turn_in_place = true;
  }
  else
  {
    delay(5);
  }
}
void Robot::mogo_receive(vector<double> f)
{

  //copy and pasted, changed to work with the attributes given by receive_data
  failed_update = 0;
  turn_in_place = false;
  chasing_mogo = true;
  flicker = 127;

  double angle_threshold = 1;
  turn_coefficient = 2;

  if (!task_exists("DEPTH")) start_task("DEPTH", Robot::check_depth);


  double lidar_depth = max(f[0], 0.2);
  double angle = f[1];

  double coefficient;

  if (abs(angle) > angle_threshold) coefficient = 300 * lidar_depth * (0.20 / seconds_per_frame);
  else if (abs(angle) < angle_threshold && chasing_mogo == true) coefficient = 600;

  heading = imu_val + angle;
  new_y = y + coefficient * cos(heading / 180 * pi);
  new_x = x - coefficient * sin(heading / 180 * pi);

  delay(5);
  turn_coefficient = 1;
}

void Robot::receive_mogo(nlohmann::json msg) {
    failed_update = 0;
    turn_in_place = false;
    chasing_mogo = true;
    flicker = 127;

    double angle_threshold = 1;
    turn_coefficient = 2;

    if (!task_exists("DEPTH")) start_task("DEPTH", Robot::check_depth);

    string msgS = msg.dump();
    std::size_t found = msgS.find(",");

    double lidar_depth = max(std::stod(msgS.substr(1, found - 1)), 0.2);
    double angle = std::stod(msgS.substr(found + 1, msgS.size() - found - 1));

    double coefficient;

    if (abs(angle) > angle_threshold) coefficient = 300 * lidar_depth * (0.20 / seconds_per_frame);
    else if (abs(angle) < angle_threshold && chasing_mogo == true) coefficient = 600;

    heading = imu_val + angle;
    new_y = y + coefficient * cos(heading / 180 * pi);
    new_x = x - coefficient * sin(heading / 180 * pi);

    delay(5);
    turn_coefficient = 1;
}




void Robot::receive_ring(nlohmann::json msg) {
    //original ring receive, technically works(?) with the new stuff i put in trt_vis
    //[unsure as of 3/27 if they do actually work in conjunction]
    turn_in_place = false;
    heading = last_heading;
    turn_coefficient = 3;
    while(abs(heading - imu_val) > 3) delay(5);

    conveyor = -127;
    string msgS = msg.dump();
    std::size_t found = msgS.find(",");

    double lidar_depth = std::stod(msgS.substr(1, found - 1));
    double angle = std::stod(msgS.substr(found + 1, msgS.size() - found - 1));
    double coefficient = lidar_depth * meters_to_inches * inches_to_encoder + 300;
    double x = gps.get_status().x;
    double y = gps.get_status().y;

    double gps_heading = gps.get_heading();
    double sin_f = sin(gps_heading)*lidar_depth+y;
    double cos_f = cos(gps_heading)*lidar_depth+x;

    if(sin_f<=-2.1*12/meters_to_inches || sin_f>=2.1*12/meters_to_inches || cos_f <= -2.1*12/meters_to_inches
    || cos_f >= 2.1*12/meters_to_inches)
    {
      delay(5);

    }
    else
    {
      double angle_threshold = 1;
      double target_heading = imu_val + angle;
      heading = target_heading;
      while (abs(imu_val - target_heading) > 3) delay(5);

      new_y = y - coefficient * cos(heading / 180 * pi);
      new_x = x + coefficient * sin(heading / 180 * pi);
      while (abs(new_y - y) > 100 or abs(new_x - x) > 100) delay(5);

      conveyor = 0;
      delay(500);

      lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#continue#true#");
      turn_in_place = true;
    }
}

void Robot::receive_fps(nlohmann::json msg){
    double seconds_per_frame = std::stod(msg.dump());
    lcd::print(7, "Seconds per frame: %f", seconds_per_frame);
    last_heading = imu_val;
    if (turn_in_place){
            heading = imu_val + 30;
    }
    if (chasing_mogo) failed_update += 1;
}


void Robot::drive(void *ptr) {
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

        bool flicker_on = master.get_digital(DIGITAL_UP);

        bool lift_up = master.get_digital(DIGITAL_LEFT);
        bool lift_down = master.get_digital(DIGITAL_RIGHT);


        if (angler_backward) angler = 40;
        else if (angler_forward) angler = -40;
        else angler = 0;

        if (angler_start_thread  && !task_exists("ANGLER")) start_task("ANGLER", Robot::depth_angler);

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
    lib7405x::Serial::Instance()->send(lib7405x::Serial::STDOUT, "#camera#l515_front#mode#true#@#");
    turn_in_place = true;
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
        lcd::print(5, "%d", int(depth_average));
        if (depth_average < 100 && depth_vals.size() == 100){
            lcd::print(4, "Here");
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

        delay(5);
    }
}


void Robot::gps_fps(void *ptr){
    while (true){
        lcd::print(5, "Y: %f - X: %f", (float)(gps.get_status().y), (float)(gps.get_status().x));
        lcd::print(6, "Heading: %f", (float)gps.get_heading());
        delay(5);
    }
}

/* new_y, new_x, and new_heading_gps are all in absolute field terms
 * new_y_gps and new_x_gps should be declared in meters
 * new_y_gps and new_x_gps are based on gps camera position not center of the bot
 * (0,0) in the center of the field in meters
 * 0 degrees is facing north (red side on left, blue side on right)
 */
void Robot::move_to_gps(void *ptr) {
    while (true)
    {
        double angle_adjust = 0;
        if (gps.get_heading()+90 >= 360) angle_adjust = -270;
        else angle_adjust = 90;

        double phi = (gps.get_heading()+angle_adjust) * pi / 180;

        double gps_error = new_heading_gps - gps.get_heading();
        double y_error = (new_y - gps.get_status().y) * meters_to_inches * inches_to_encoder;
        double x_error = (new_x - gps.get_status().x) * meters_to_inches * inches_to_encoder;

        double power = power_PD.get_value(y_error * std::cos(phi) + x_error * std::sin(phi));
        double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error * std::sin(phi));
        double turn = turn_PD.get_value(gps_error);

        mecanum(power, strafe, turn, 127);

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


void Robot::display(void *ptr){
    while (true){
        lcd::print(1, "X: %d, Y: %d", (int)x, (int)y);
        lcd::print(2, "nX: %d, nY: %d", (int)new_x, (int)new_y);
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
