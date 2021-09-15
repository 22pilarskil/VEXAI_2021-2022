#include <math.h>

static int sgn(double n);

static double sin_degrees(double  d);
static double cos_degrees(double  d);
static double tan_degrees(double d);
static double clamp(double number, double lowerBound, double upperBound);
static double truncate(double d, int place);
static bool equals(double d, double e);
static bool equals(double d, double e, double tolerance);
static double pow_retaining_sign(double d, double power);
static double wrap(double d, double min, double max);
static double standard_deviation(int[] values);
static double map(double value, double lower1, double upper1, double lower2, double upper2);
static double normalize(double angle);
static double to_radians(double degrees);