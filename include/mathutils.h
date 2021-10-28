#ifndef INC_MATH_UTILS_H
#define INC_MATH_UTILS_H

#include <cmath>
#include <vector>

class mathutils {
private:
    static constexpr double EPSILON = 0.000001;
    static constexpr double TO_RAD = M_PI / 180;

public:
    static int sgn(double n);

    static double sinDegrees(double deg);

    static double cosDegrees(double deg);

    static double tanDegrees(double deg);

    static double clamp(double n, double low, double high);

    static double truncate(double n, int place);

    static bool equals(double n, double m, double tolerance = EPSILON);

    static double powRetainingSign(double base, double exp);

    static double wrap(double n, double min, double max);

    static double standardDeviation(const std::vector<int>& values);

    static double map(double n, double low1, double high1, double low2, double high2);

    static double maxArr(const std::vector<double>& vec);

    static double normalize(double angle);
};

#endif//INC_MATH_UTILS_H
