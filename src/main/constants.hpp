#pragma once

#include <units/angle.h>
#include <units/time.h>


constexpr units::second_t k_control_dt = 0.01_s;
constexpr double k_control_dts = 0.01;


// pivot angle convention - 0 degrees is horizontal line to center of mass,
// deployed is close to 0 degrees
constexpr units::degree_t k_intake_deployed = -14_deg;
constexpr units::degree_t k_intake_hardstop = k_intake_deployed + 129.5_deg;
constexpr units::degree_t k_intake_stowed = k_intake_hardstop - 2_deg;
constexpr units::degree_t k_intake_depot = k_intake_deployed + 5_deg;
constexpr units::degree_t k_intake_spring_center = k_intake_deployed + 75_deg;
constexpr units::degree_t k_intake_range = k_intake_hardstop - k_intake_deployed;

constexpr double k_intake_succ_speed = 0.8;
constexpr double k_intake_vomit_speed = -0.8;
constexpr double k_intake_roller_push_hopper_speed = 0.1;



constexpr double k_swerve_jerk_threshold = 180;

//shooter constants
constexpr double turret_turn_min_angle = 558.0;
constexpr double turret_turn_max_angle = 9.0;
constexpr double hood_min_angle = 66;
constexpr double hood_max_angle = 45;

constexpr double shotmap[] = 
//distance_rpm_and_hood_angle_for_shotmap_in_that order 
    {
        60,         66.0067,     1500.0067,
        75,         66.0067,     1550.0067,
        85,         66.0067,     1600.0067,
        95,         65.0067,     1700.0067,
        105,        64.0067,     1750.0067,
        115,        63.0067,     1800.0067,
        125,        62.0067,     1825.0067,
        135,        61.0067,     1835.0067,
        145,        60.0067,     1860.0067,
        155,        59.0067,     1910.0067,
        165,        59.0067,     1950.0067,
        175,        59.0067,     2010.0067,
        185,        59.0067,     2075.0067,
        195,        59.0067,     2110.0067,
        205,        59.0067,     2150.0067,
        210,        59.0067,     2200.0067,
    };


const blue_vec2 k_hub{ 4.641, 4.023 };
