#ifndef _Data
#define _Data
#include <vector>
#include "system/json.hpp"
using namespace std;
using namespace pros;

class Data{
	public:
		static bool invalid_det(std::vector<float> det, double cur_x_gps, double cur_y_gps, double gps_heading);
		static vector<vector<float>> pred_id(vector<vector<float>> pred, int id);
		static vector<vector<float>> get_pred(nlohmann::json msg);
};
#endif