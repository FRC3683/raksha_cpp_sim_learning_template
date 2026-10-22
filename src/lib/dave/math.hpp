#pragma once

#include <units/angle.h>

#define PI 3.14159265359
#define HALF_PI 1.57079632679

enum direction {
    ANTI_CLOCKWISE = 1,
    CLOCKWISE = -1,
};

double int_pow( double d, int p );
double deadbanded( double d, double deadband, double max_value );
double move_toward( double start, double target, double max_delta );
double interpolate( double start, double end, double alpha );
double unlerp( double start, double end, double value );
double remap( double value, double old_start, double old_end, double new_start, double new_end );
double signum( double value );
double sin( units::degree_t theta );
double cos( units::degree_t theta );
units::radian_t arcsin( double d );
units::radian_t arccos( double d );
units::radian_t arctan( double y, double x );
units::radian_t wrap_angle( units::radian_t angle );

