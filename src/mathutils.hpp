#include "mathutils.hpp"
//courtesy of the legend Eric Dong
int mathutils::sgn(double n) {
    return (n > 0) - (n < 0);
}

double mathutils::sinDegrees(double deg) {
    return std::sin(deg * TO_RAD);
}

double mathutils::cosDegrees(double deg) {
    return std::cos(deg * TO_RAD);
}

double mathutils::tanDegrees(double deg) {
    return std::tan(deg * TO_RAD);
}

double mathutils::clamp(double n, double low, double high) {
    return std::max(low, std::min(high, n));
}

double mathutils::truncate(double n, int place) {
    return ((double) ((int) (n * place))) / place;
}

bool mathutils::equals(double n, double m, double tolerance) {
    return std::abs(n - m) < tolerance;
}

double mathutils::powRetainingSign(double base, double exp) {
    return std::copysign(std::pow(base, exp), base);
}

double mathutils::wrap(double n, double min, double max) {
    if (!equals(n, min) && n < min) {
        n = max - (min - n);
    } else if (!equals(n, max) && n > max) {
        n = min + (n - max);
    }
    return n;
}

double mathutils::standardDeviation(const std::vector<int>& values) {
    double mean = 0;
    for (auto const& i : values) {
        mean += i;
    }
    mean /= values.size();
    double sd = 0;
    for (auto const& i : values) {
        sd += std::pow(i - mean, 2);
    }
    sd /= values.size();
    return std::sqrt(sd);
}

double mathutils::map(double n, double low1, double high1, double low2, double high2) {
    return ((n - low1) * (high2 - low2) / (high1 - low1)) + low2;
}

double mathutils::maxArr(const std::vector<double>& vec) {
    double max = std::numeric_limits<double>::min();
    for (const auto& i : vec) {
        max = std::max(i, max);
    }
    return max;
}

double mathutils::normalize(double angle) {
    return fmod((fmod(angle, 2 * M_PI) + 2 * M_PI), (2 * M_PI));
}
