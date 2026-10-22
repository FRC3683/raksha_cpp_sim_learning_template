#pragma once

#include <units/angle.h>
#include <units/acceleration.h>

#include "dave/swerve/swerve_output.hpp"

struct swerve_request {
    field_relative_output target_output = field_relative_output{};
    units::meters_per_second_squared_t max_linear_accel = 10_mps_sq;
    units::radians_per_second_squared_t max_angular_accel = 900_deg_per_s_sq;
    double turn_bias = 0.5;
    vec2 center_of_rotation = vec2::zero;
};
