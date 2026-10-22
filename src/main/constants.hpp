#pragma once

#include <units/angle.h>
#include <units/time.h>


constexpr units::second_t k_control_dt = 0.01_s;
constexpr double k_control_dts = 0.01;


constexpr double k_swerve_jerk_threshold = 180;

const blue_vec2 k_hub{ 4.641, 4.023 };
