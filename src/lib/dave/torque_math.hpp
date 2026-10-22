#pragma once

#include "dave/vector.hpp"
#include "dave/swerve/swerve_output.hpp"



double limit_torque( double target_output, double current_speed, double free_speed, double torque_limit );
vec2 limit_torque( vec2 target_output, vec2 current_speeds, double free_speed, double torque_limit, units::meters_per_second_squared_t max_centripital_accel, units::second_t delta_time );

double get_applied_torque( double applied_output, double current_speed, double free_speed );