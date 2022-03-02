#include <iostream>
#include "Grid.cpp"
#include <vector>
#include <cmath>

class GridMapper
{
private:
    Grid* g = new Grid();
public:
    void map(double robot_pos[], std::map<std::string, std::vector<double*>> objects) // Needs to take in everything we see. We need to decide what format our data will be before this is coded.'
    {

        // This current one is just for testing. For the actual version, all objects will be parsed and put into the grid.
        // I'm going to be working under the assumption that we can get the position, true heading and distance / angle of all objects we see.

        // For this, I will be using a sample dictionary.
        for (auto const& x : objects)
        {
            std::string name = x.first;
            std::vector<double*> pos_list = x.second;

            for (int i = 0; i < pos_list.size(); i++)
            {
                int truePos[2];
                truePos[0] = robot_pos[0] + pos_list[i][0] * cos(robot_pos[2] + pos_list[i][1]);
                truePos[1] = robot_pos[1] + pos_list[i][0] * sin(robot_pos[2] + pos_list[i][1]);

                g->add(name, truePos);
            }
        }
    }


    std::vector<int> viewed_boxes(double robot_pos[]) {
        std::vector<int> viewed_boxes;

        //btw to calculate each box, i'm using the bottom right corner of each box because it is easiest to calculate the box number from its coordinates


        if (robot_pos[2] > 35 / 180.0 * M_PI && robot_pos[2] < (180 - 35) / 180.0 * M_PI) { // facing up
            int range_prev[] = { 0,0 };
            for (int y = ceil(robot_pos[1]); y <= g->get_y_length(); y++) {
                int x_left = ceil((y - robot_pos[1]) / tan(robot_pos[2] + 35 / 180.0 * M_PI) + robot_pos[0]);
                int x_right = floor((y - robot_pos[1]) / tan(robot_pos[2] - 35 / 180.0 * M_PI) + robot_pos[0]);
              x_left = fmax(0, x_left);
              x_right = fmin(6, x_right);

                /*int range[] = { 0,0 };
                range[0] = x_left;
                range[1] = x_right;*/

                if (x_right > x_left && y < g->get_y_length()) //checks there is at least two valid x values and that we still have another y row above us
                {
                    for (x_left = x_left + 1; x_left <= x_right; x_left++) //loop thru each x pair
                    {
                        int box = (6 * y) + x_left;
                        if (box > 36) 
                        {
                          std::cout << box << std::endl;
                        }
                        viewed_boxes.push_back(box);
                    }
                }

                range_prev[0] = x_left;
                range_prev[1] = x_right;

            }
        }

        //facing right
        else if (robot_pos[2] < 35.0 / 180 * M_PI && (robot_pos[2] > (360.0 - 35) / 180 * M_PI || robot_pos[2] >= 0)) {
            int range_prev[] = { 0,0 };
            for (int x = ceil(robot_pos[0]); x <= g->get_x_length(); x++) {
                int y_top = floor(tan(robot_pos[2] + 35.0 / 180 * M_PI) * (x - robot_pos[0]) + robot_pos[1]);
                int y_bot = ceil(tan(robot_pos[2] - 35.0 / 180 * M_PI) * (x - robot_pos[0]) + robot_pos[1]);
              y_bot = fmax(0, y_bot);
              y_top = fmin(6, y_top);

                /*int range[] = { 0,0 };
                range[0] = y_top;
                range[1] = y_bot;*/

                if (y_top > y_bot && x < g->get_x_length()) //checks there is at least two valid y values and we still have another x column to the right
                {
                    for (y_bot = y_bot + 1; y_bot < y_top; y_bot++) //loops through each y pair
                    {
                        int box = (6 * y_bot) + x;
                        viewed_boxes.push_back(box);
                    }
                }

                range_prev[0] = y_top;
                range_prev[1] = y_bot;
            }
        }

        //facing down
        else if (robot_pos[2] > (180.0 + 35) / 180 * M_PI && robot_pos[2] < (360.0 - 35) / 180 * M_PI) {
            int range_prev[] = { 0,0 };
            for (int y = floor(robot_pos[1]); y <= g->get_y_length(); y++) {
                int x_right = floor((y - robot_pos[1]) / tan(robot_pos[2] + 35.0 / 180 * M_PI) + robot_pos[0]);
                int x_left = ceil((y - robot_pos[1]) / tan(robot_pos[2] - 35.0 / 180 * M_PI) + robot_pos[0]);
              x_left = fmax(0, x_left);
              x_right = fmin(6, x_right);

               /* int range[] = { 0,0 };
                range[0] = x_left;
                range[1] = x_right;*/

                if (x_right > x_left && y > g->get_y_length()) // checks we have at least two valid x values and another y row below us
                {
                    for (x_left = x_left + 1; x_left < x_right; x_left++) //loops thru each x pair
                    {
                        int box = (6 * y) + x_left;
                        std::cout << box << " is VALID" << std::endl;
                        viewed_boxes.push_back(box);
                    }
                }

                range_prev[0] = x_left;
                range_prev[1] = x_right;
            }
        }

        //facing left
        else if (robot_pos[2] > (180.0 - 35) / 180 * M_PI && robot_pos[2] < (180.0 + 35) / 180 * M_PI) {
            int range_prev[] = { 0,0 };
            for (int x = floor(robot_pos[0]); x <= g->get_x_length(); x++) {
                int y_bot = ceil(tan(robot_pos[2] + 35.0 / 180 * M_PI) * (x - robot_pos[0]) + robot_pos[1]);
                int y_top = floor(tan(robot_pos[2] - 35.0 / 180 * M_PI) * (x - robot_pos[0]) + robot_pos[1]);
              y_bot = fmax(0, y_bot);
              y_top = fmin(6, y_top);

                /*int range[] = { 0,0 };
                range[0] = y_top;
                range[1] = y_bot;*/

                if (y_top > y_bot && x < g->get_x_length()) //checks we have at least two valid y values and another x column to our left
                {
                    for (y_bot++; y_bot < y_top; y_bot++)
                    {
                        int box = (6 * y_bot) + x;
                        viewed_boxes.push_back(box);
                    }
                }

                range_prev[0] = y_top;
                range_prev[1] = y_bot;
            }
        }
        else {

        }

        // for (int i : viewed_boxes) {
        //     std::cout << "viewed: " << i << std::endl;
        // }

        /* A point is valid if:
        *  The y value is between the y values of both lines
        *  The x value is between the x values of both fines
        */

        return viewed_boxes;
    }
};

