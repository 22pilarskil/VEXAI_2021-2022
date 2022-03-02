#include <map>
#include <string>
#include <list>

class Grid
{
private:
    std::map<std::string, std::list<int*>> object_to_pos;
    std::map<int*, std::list<std::string>> pos_to_object;
    int x_length = 6;
    int y_length = 6;

public:
    int get_x_length() {
        return x_length;
    }
    int get_y_length() {
        return y_length;
    }
    std::list<int*> get(std::string name) {
        return object_to_pos[name];
    }
    std::list<std::string> get(int* pos) {
        return pos_to_object[pos];
    }

    void add(std::string name, int* pos)
    {
        object_to_pos[name].push_back(pos);
        pos_to_object[pos].push_back(name);
    }

    void remove(std::string name, int* pos) {
        object_to_pos[name].remove(pos);
        pos_to_object[pos].remove(name);
    }
};
