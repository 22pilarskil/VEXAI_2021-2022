#include "main.h"
#include "system/Data.h"
#include <vector>
#include "system/json.hpp"
#include <cmath>
using namespace std;
using namespace pros;

const double pi = 3.141592653589793238;
const double meters_to_inches = 39.3701;

vector<vector<float>> Data::pred_id(vector<vector<float>> pred, int id)
{
    vector<vector<float>> tp;
    for(vector<float> curr:pred){
        if(curr[2]==id){
            tp.push_back(curr);
        }
    }
    return tp;
}

vector<vector<float>> Data::get_pred(nlohmann::json msg){
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

bool Data::invalid_det(std::vector<float> det, double cur_x, double cur_y, double gps_heading)
{
    gps_heading = (gps_heading)*(pi/180);//to radians
    double lidar_depth = det[0];
    double angle = det[1];
    double final_angle = gps_heading - (angle*pi/180);
    final_angle < 0 ? final_angle = 2*pi + final_angle : final_angle = final_angle;//angle that connects ring and bot
    final_angle > 2*pi ? final_angle = final_angle - 2*pi : final_angle = final_angle;

    double ring_y = sin(final_angle)*lidar_depth+cur_y;

    double ring_x = cos(final_angle)*lidar_depth+cur_x;
    lcd::print(2, "%f", gps_heading);
    //lcd::print(4, "%f, %f", (float)ring_x, (float)ring_y);
    //lcd::print(5, "%f %f %f", (float)lidar_depth, (float) angle, (float)final_angle*180/pi);


    double temp_dist = 6;//inches away from wall
    double min_wall_distance = (70.5-temp_dist) / meters_to_inches;

    double balance_threshold = 6; //how close we want to allow our bot to get to the balance in inches
    double balance_corner_y = (27 + balance_threshold) / meters_to_inches;
    double balance_corner_x = ((70.5-23) - balance_threshold) / meters_to_inches;

    // There is probably a more concise way to write this but it works so whatever.

    bool under_balance = (abs(ring_y) <= balance_corner_y) && (abs(ring_x) >= balance_corner_x);

    bool too_close_to_wall = abs(ring_y)>=min_wall_distance || abs(ring_x) >= min_wall_distance;

    if(!under_balance && !too_close_to_wall)
    {
      //define the line that connects the points in mx+b

      //lcd::print(3, "%s", "LINE PROGRAM");

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
    else{//lcd::print(3, "%s", "NOT LINE PROGRAM");
    return true;}
}
