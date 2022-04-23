#include <map>
#include <string>
#include <list>
#include <cmath>
#include <vector>

class Grid
{
  private: 
    std::map<std::string, std::map<int, int>> object_to_pos;
    std::map<int, std::map<std::string, int>> pos_to_object;

// {"yellow mogo": {1: 2, 2: 2, 3: 0}, 
//  "ring": {1: 6, 2: 0, 3: 2}}
// {1: {"yellow mogo": 2, "ring": 6}, 
//  2: {"yellow mogo": 2, "ring": 0}}

    int x_length = 6;
    int y_length = 6;

  public:
    int get_x_length() {
      return x_length;
    }
    int get_y_length() {
      return y_length;
    }
    std::map<int, int> get(std::string name) {
      return object_to_pos[name];
    }

    std::map<std::string, int> get(double *pos) {
      return pos_to_object[1 + floor(pos[0] / 24.0) + x_length * (ceil(pos[1] / 24.0))];
    }

    std::map<std::string, int> get(int box) {
      return pos_to_object[box];
    }
  
    void add(std::string name, double *pos, std::vector<int> validBoxes)
    {
      int box = 1 + floor(pos[0] / 24.0) + x_length * (6 - ceil(pos[1] / 24.0));

      for (int i = 0; i < validBoxes.size(); i++)
      {
        if (box == validBoxes[i])
        {
            object_to_pos[name][box]++;
            pos_to_object[box][name]++;
        }
      }
    }

    void add(std::string name, int box)
    {
      object_to_pos[name][box]++;
      pos_to_object[box][name]++;
    }
  
    void remove(std::string name, double *pos) {
      int box = 1 + floor(pos[0] / 12.0) + x_length *  (ceil(pos[1] / 24.0));
      object_to_pos[name][box]--;
      pos_to_object[box][name]--;
    }

    void remove(std::string name, int box) {
      object_to_pos[name][box]--;
      pos_to_object[box][name]--;
    }

    void remove(int box) {
      for(std::pair<std::string, int> obj : pos_to_object[box]) {
        pos_to_object[box][obj.first] = 0;
        object_to_pos[obj.first][box] = 0;
      }
    }

    std::map<std::string, int> getBox(int boxNumber) {
      return pos_to_object[boxNumber];
    }

    std::map<int, int> getPos(std::string objName) {
      return object_to_pos[objName];
    }
};

