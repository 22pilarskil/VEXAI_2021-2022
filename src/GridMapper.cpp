#include <iostream>
#include "Grid.cpp"
#include <vector>
#include <cmath>
#include "main.h"
using namespace pros;

class GridMapper {
  private:
    Grid *g = new Grid();
  public:
    void map(double robot_pos[], std::map<std::string, std::vector<std::vector<double>>> objects) // Needs to take in everything we see. We need to decide what format our data will be before this is coded.
    {
      // This current one is just for testing. For the actual version, all objects will be parsed and put into the grid.
      // I'm going to be working under the assumption that we can get the position, true heading and distance / angle of all objects we see.

      // For this, I will be using a sample dictionary.
      lcd::print(6, (std::to_string(objects["mogo"][0][1])).c_str());
      std::vector<int> validBoxes = remove_viewed(robot_pos);
      // lcd::print(6, (std::to_string(objects["mogo"][0][1])).c_str());
      robot_pos[0] *= 24;
      robot_pos[1] *= 24;
      for (auto const& x : objects) {
        std::string name = x.first;
        std::vector<std::vector<double>> pos_list = x.second;
        for (int i = 0; i < pos_list.size(); i++)
        {
          double truePos[2];
          truePos[0] = robot_pos[0] * 24 + pos_list[i][0] * cos(robot_pos[2] + pos_list[i][1]);
          truePos[1] = robot_pos[1] * 24 + pos_list[i][0] * sin(robot_pos[2] + pos_list[i][1]);
          // lcd::print(6, (std::to_string(robot_pos[0]) + " " + std::to_string(pos_list[i][0])).c_str());
          g->add(name, truePos, validBoxes);
        }
      }
    }


    std::vector<int> remove_viewed(double robot_pos[]) {
      std::vector<int> viewed_boxes;

      robot_pos[0] = robot_pos[0] / 24.0;
      robot_pos[1] = robot_pos[1] / 24.0;


      if (robot_pos[2] > 35/180.0*M_PI && robot_pos[2] < (180-35)/180.0*M_PI) { // Facing Up
        int range_prev[] = {0,0};
        for (int y = ceil(robot_pos[1]); y <= g->get_y_length(); y++) {
          int x_left = ceil((y - robot_pos[1]) / tan(robot_pos[2] + 35/180.0*M_PI) + robot_pos[0]);
          int x_right = floor((y - robot_pos[1]) / tan(robot_pos[2] - 35/180.0*M_PI) + robot_pos[0]);

          int range[] = {0,0};
          range[0] = std::max(0, x_left);
          range[1] = std::min(6, x_right);

          int box_range[] = {std::max(range[0], range_prev[0]), std::min(range[1], range_prev[1])};

          if (!(box_range[0] >= box_range[1])) {
            for (int i = box_range[0]; i < box_range[1]; i++) {
              viewed_boxes.push_back(i + 1 + (6 - y) * g->get_x_length());
            }
          }

          range_prev[0] = range[0];
          range_prev[1] = range[1];

        }
      }

      else if (robot_pos[2] < 35.0/180*M_PI && (robot_pos[2] > (360.0-35)/180*M_PI || robot_pos[2] >= 0)) { // Facing Right
        int range_prev[] = {0,0};
        for (int x = ceil(robot_pos[0]); x <= g->get_x_length(); x++) {
          int y_top = floor(tan(robot_pos[2] + 35.0/180*M_PI) * (x - robot_pos[0]) + robot_pos[1]);
          int y_bot = ceil(tan(robot_pos[2] - 35.0/180*M_PI) * (x - robot_pos[0]) + robot_pos[1]);

          int range[] = {0,0};
          range[0] = std::max(0, y_top);
          range[1] = std::min(6, y_bot);

          int box_range[] = {std::min(range[0], range_prev[0]), std::max(range[1], range_prev[1])};

          if (!(box_range[1] >= box_range[0])) {
            for (int i = box_range[0]; i > box_range[1]; i--) {
              viewed_boxes.push_back(x + (6 - i) * (g->get_x_length()));
            }
          }

          range_prev[0] = range[0];
          range_prev[1] = range[1];
        }
      }

      else if (robot_pos[2] > (180.0+35)/180*M_PI && robot_pos[2] < (360.0-35)/180*M_PI) { // Facing Down
        int range_prev[] = {0,0};
        for (int y = floor(robot_pos[1]); y >= 0; y--) {
          int x_right = floor((y - robot_pos[1]) / tan(robot_pos[2] + 35.0/180*M_PI) + robot_pos[0]);
          int x_left = ceil((y - robot_pos[1]) / tan(robot_pos[2] - 35.0/180*M_PI) + robot_pos[0]);

          int range[] = {0,0};
          range[0] = std::max(0, x_left);
          range[1] = std::min(6, x_right);

          int box_range[] = {std::max(range[0], range_prev[0]), std::min(range[1], range_prev[1])};

          if (!(box_range[0] >= box_range[1])) {
            for (int i = box_range[0]; i < box_range[1]; i++) {
              viewed_boxes.push_back(i + 1 + (6 - (y+1)) * g->get_x_length());
            }
          }

          range_prev[0] = range[0];
          range_prev[1] = range[1];
        }
      }

      else if (robot_pos[2] > (180.0-35)/180*M_PI && robot_pos[2] < (180.0+35)/180*M_PI) { // Facing Left
        int range_prev[] = {0,0};
        for (int x = floor(robot_pos[0]); x >= 0; x--) {
          int y_bot = ceil(tan(robot_pos[2] + 35.0/180*M_PI) * (x - robot_pos[0]) + robot_pos[1]);
          int y_top = floor(tan(robot_pos[2] - 35.0/180*M_PI) * (x - robot_pos[0]) + robot_pos[1]);

          int range[] = {0,0};
          range[0] = std::max(0, y_top);
          range[1] = std::min(6, y_bot);

          int box_range[] = {std::min(range[0], range_prev[0]), std::max(range[1], range_prev[1])};

          if (!(box_range[1] >= box_range[0])) {
            for (int i = box_range[0]; i > box_range[1]; i--) {
              viewed_boxes.push_back(x + 1 + (6 - i) * (g->get_x_length()));
            }
          }

          range_prev[0] = range[0];
          range_prev[1] = range[1];
        }
      }

      for(int i : viewed_boxes){
        std::cout << "Box: " + std::to_string(i) << std::endl;
        g->remove(i);
      }

      return viewed_boxes;
    }

    std::map<std::string, int> getBox(int boxNumber) {
      return g->getBox(boxNumber);
    }

    std::map<int, int> getPos(std::string objName) {
      return g->getPos(objName);
    }

    void add(std::string name, int box)
    {
      g->add(name,box);
    }
};
