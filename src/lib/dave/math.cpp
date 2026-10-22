#include "dave/math.hpp"

#include <frc/MathUtil.h>
#include <cmath>


double int_pow( double d, int p ) {
    double r = 1;
    for( ; p > 0; --p ) {
        r *= d;
    }
    return r;
}

int int_factorial( int f ) {
    int r = 1;
    for( ; f > 0; --f ) {
        r *= f;
    }
    return r;
}

double deadbanded( double d, double deadband, double max_value ) {
    if( std::abs( d ) < deadband ) {
        return 0;
    }
    return signum(d) * remap( std::abs(d), deadband, max_value, 0, max_value );
}

double move_toward( double start, double target, double max_delta ) {
    if( std::abs(target - start) <= max_delta ) {
        return target;
    }
    if( start < target ) {
        return start + max_delta;
    }
    return start - max_delta;
}

double interpolate( double start, double end, double alpha ) {
    return start + ( ( end - start ) * std::clamp( alpha, 0.0, 1.0 ) );
}

double unlerp( double start, double end, double value ) {
    return std::clamp( ( value - start ) / ( end - start ), 0.0, 1.0 );
}

double remap( double value, double old_start, double old_end, double new_start, double new_end ) {
    return interpolate( new_start, new_end, unlerp( old_start, old_end, value ) );
}

double signum( double value ) {
    if( value < 0 ) {
        return -1;
    }
    if( value > 0 ) {
        return 1;
    }
    return 0;
}

double sin( units::degree_t theta ) {
    return std::sin( units::radian_t{ theta }.value() );
}

double cos( units::degree_t theta ) {
    return std::cos( units::radian_t{ theta }.value() );
}

units::radian_t arcsin( double d ) {
    return units::radian_t{ std::asin( d ) };
}

units::radian_t arccos( double d ) {
    return units::radian_t{ std::acos( d ) };
}

units::radian_t arctan( double y, double x ) {
    return units::radian_t{ std::atan2( y, x ) };
}

units::radian_t wrap_angle( units::radian_t angle ) {
    return units::radian_t{ frc::InputModulus( angle.value(), -PI, PI ) };
}
