#include <math.h>

static int sgn(double n) {
    if (MathUtils::equals(n, 0)) {
        return 0;
    } else if (n > 0) {
        return 1;
    } else {
        return -1;
    }
}

static double sin_degrees(double  d) {
    return sin(to_radians(d));
}
static double cos_degrees(double  d) {
    return cos(to_radians(d));
}
static double tan_degrees(double d) { return tan(to_radians(d)); }
static double clamp(double number, double lowerBound, double upperBound) {
    return fmax(lowerBound, fmin(upperBound, number));
}
static double truncate(double d, int place) {
    return (double) ((int)(d*place)) / place;
}
static bool equals(double d, double e) {
    return equals(d,e,epsilon);
}
static bool equals(double d, double e, double tolerance) {
    if (Double.isNaN(d) || Double.isNaN(e))
        return false;
    return abs(d-e) < tolerance;
}
static double pow_retaining_sign(double d, double power) {
    if (abs(d) < 1e-14) {
        return 0;
    }
    return copysign(pow(abs(d), power), d);
}

static double wrap(double d, double min, double max) {
    if (!equals(d, min) &&d < min) {
        d = max - (min - d);
    } else if (!equals(d, max) && d > max) {
        d = min + (d - max);
    }
    return d;
}

static double standard_deviation(int[] values) {
    int len = values.length;
    double mean = 0;
    for (int i = 0; i < len; i++) {
        mean += values[i];
    }
    mean /= len;
    double stddev = 0;
    for (int i = 0; i < len; i++) {
        double diff = values[i] - mean;
        stddev += diff*diff;
    }
    stddev /= len;
    return sqrt(stddev);
}
static double map(double value, double lower1, double upper1, double lower2, double upper2) {
    return ((value-lower1)*(upper2-lower2)/(upper1-lower1))+lower2;
}
static double max_array(double[] array) {
    double max = abs(array[0]);
    for (double a : array) {
        if (abs(a) > max) {
            max = abs(a);
        }
    }
    return max;
}
static double normalize(double angle) {
    double newAngle = angle%(2*M_PI);
    newAngle = (newAngle + 2*M_PI) % (2*M_PI);

    return newAngle;
}

static double to_radians(double degrees) {
    return degrees*M_PI/180;
}