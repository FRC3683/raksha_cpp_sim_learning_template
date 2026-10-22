#include <dave/torque_math.hpp>

#include <frc/smartdashboard/SmartDashboard.h>

double limit_torque( double target_output, double current_speed, double free_speed, double torque_limit ) {
    double neutral = ( current_speed / free_speed );
    return std::clamp( target_output, neutral - torque_limit, neutral + torque_limit );
}

vec2 limit_torque( vec2 target_output, vec2 current_speeds, double free_speed, double torque_limit, units::meters_per_second_squared_t max_centripital_accel, units::second_t delta_time ) {
    double current_speed = current_speeds.length();

    if( current_speed < 0.15 ) {
        double l = limit_torque( target_output.length(), current_speeds.length(), free_speed, torque_limit );
        return vec2{ l, target_output.theta() };
    }

    if( target_output.length_squared() < 0.001 ) {
        double l = limit_torque( 0, current_speed, free_speed, torque_limit );
        return vec2{ l, current_speeds.theta() };
    }

    // a_c = v^2 / r
    double min_arc_radius = ( current_speed * current_speed ) / max_centripital_accel.value();

    units::radians_per_second_t slip_angle_rate = 15_rad_per_s * current_speed / min_arc_radius; // * 10 to compensate for phase delay
    units::radian_t max_slip_angle = slip_angle_rate * delta_time;

    frc::SmartDashboard::PutNumber( "debug/slip_angle_rate", units::degrees_per_second_t{ slip_angle_rate }.value() );
    frc::SmartDashboard::PutNumber( "debug/slip_angle", units::degree_t{ max_slip_angle }.value() );

    double hat_dot = target_output.hat().dot( current_speeds.hat() );

    double lim = ( current_speeds / free_speed ).moved_towards( target_output, torque_limit ).length();
    if( hat_dot < -0.45 ) {
        target_output = -target_output;
        lim = hat_dot;
    }
    double l = limit_torque( lim, current_speeds.length(), free_speed, torque_limit );
    return current_speeds.rotated_towards( target_output, max_slip_angle ).hat() * l;
}

double get_applied_torque( double applied_output, double current_speed, double free_speed ) {
    double neutral = ( current_speed / free_speed );
    return applied_output - neutral;
}
