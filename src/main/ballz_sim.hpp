#pragma once

#include <random> // for ball sim
#include <vector>

#include <frc/Timer.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "dave/vector.hpp"
#include "dave/units_common.hpp"
#include "dave/filters/hysteresis.hpp"

#include "subsystems/swerve.hpp"

constexpr units::meter_t k_flywheel_radius = 2_in;
constexpr units::kilogram_square_meter_t k_flywheel_moi = 2.05 * 0.00117056_kg_sq_m;
constexpr units::kilogram_t k_ball_heft = 0.48_lb;
constexpr units::meter_t k_ball_radius = 3_in;
constexpr units::kilogram_square_meter_t k_ball_moi = (2.0/5.0) * k_ball_heft * k_ball_radius * k_ball_radius;

std::vector<frc::Pose3d> simulated_balls_positions;
std::vector<vec3> ball_vels;

int randi( int min, int max ) {
    // Formula for inclusive bounds [min, max]
    return min + std::rand() % (max - min + 1);
}

void sim_spawn_ball( vec3 origin, vec3 velocity ) {
    static int balls_shot = 0;
    ++balls_shot;
    frc::SmartDashboard::PutNumber( "ballz shot", balls_shot );
    simulated_balls_positions.push_back( frc::Pose3d{ origin.t(), frc::Rotation3d{} } );
    ball_vels.push_back( velocity );
}

// returns resultant flywheel rpm
void sim_shoot_ball( frc::sim::DCMotorSim &flywheel_sim_model, frc::sim::DCMotorSim &hood_sim_model, frc::sim::DCMotorSim &owl_sim_model ) {
    units::radians_per_second_t Wi = flywheel_sim_model.GetAngularVelocity();
    units::meters_per_second_t Vi = 72_in / 1_s;

    units::energy::joule_t j1 = k_ball_heft * Vi * Vi;
    units::energy::joule_t j2 = units::energy::joule_t{ k_flywheel_moi.value() * Wi.value() * Wi.value() };
    auto i = k_flywheel_moi;
    auto ii = k_ball_heft * k_flywheel_radius * k_flywheel_radius * (3.0/4.0) * (3.0/4.0);
    auto iii = k_ball_moi * k_flywheel_radius * k_flywheel_radius / ( 4 * k_ball_radius * k_ball_radius );
    auto Wf = units::math::sqrt( ( j1 + j2 ) / ( i + ii + iii ) );

    units::meters_per_second_t ball_vel = 0.77067 * ( Wf * k_flywheel_radius );

    vec3 shot_velocity = vec3{ ball_vel.value(), 0, 0 }
        .rotated_around_y( -hood_sim_model.GetAngularPosition() )
        .rotated_around_z( owl_sim_model.GetAngularPosition() + swerve::get_heading() );

    sim_spawn_ball( vec3{ swerve::get_vec2(), 0.45 }, shot_velocity + vec3{ swerve::get_field_speeds().xy_output } );

    flywheel_sim_model.SetAngularVelocity( 1_rad * Wf );
}


void simulate_balls( frc::sim::DCMotorSim &flywheel_sim_model, frc::sim::DCMotorSim &feed_sim_model, frc::sim::DCMotorSim &hood_sim_model, frc::sim::DCMotorSim &owl_sim_model ) {
    static int scored = 0;
    static bool feeding_ballz = false;
    static units::second_t last_shot_time = 0_s;

    units::second_t curtime = frc::Timer::GetFPGATimestamp();

    feeding_ballz = ( feed_sim_model.GetAngularVelocity() > 100_rpm );

    if( feeding_ballz && curtime - last_shot_time > 125_ms ) {
        last_shot_time = curtime + units::millisecond_t{ (double)randi( -50, 50 ) };
        sim_shoot_ball( flywheel_sim_model, hood_sim_model, owl_sim_model );
    }

    for( int i=0; i < simulated_balls_positions.size(); ++i ) {
        vec3 pos = simulated_balls_positions[i].Translation();
        ball_vels[i] += vec3{ 0, 0, -9.8 * 0.01 };
        pos += ball_vels[i] * 0.01;
        simulated_balls_positions[i] = frc::Pose3d{ pos.t(), frc::Rotation3d{} };

        if( (pos.xy() - (vec2)k_hub).length_squared() < 0.5588*0.5588 && ball_vels[i].z < -1 && pos.z > 1.4351 && pos.z < 1.8288 ) {
            ++scored;
            frc::SmartDashboard::PutNumber( "ballz in HUB", scored );
            pos.z = -1;
        }

        if( pos.z < 0 ) {
            simulated_balls_positions[i] = simulated_balls_positions[simulated_balls_positions.size()-1];
            ball_vels[i] = ball_vels[ball_vels.size()-1];
            simulated_balls_positions.pop_back();
            ball_vels.pop_back();
        }
    }
}