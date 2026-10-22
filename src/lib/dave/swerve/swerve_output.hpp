#pragma once

#include <units/angle.h>

#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>

#include "dave/math.hpp"
#include "dave/vector.hpp"

template<bool is_field_relative>
struct swerve_output {
    vec2 xy_output = vec2::zero;
    double omega_output = 0;

    swerve_output<is_field_relative> deadbanded( double xy_deadband, double theta_deadband ) const;
    swerve_output<false> to_robot_relative( units::radian_t robot_heading ) const;
    swerve_output<true> to_field_relative( units::radian_t robot_heading ) const;
    swerve_output<is_field_relative> move_towards( const vec2 &target, const double max_delta ) const;
    swerve_output<is_field_relative> move_towards( const swerve_output<is_field_relative> &target, const double max_xy_delta, const double max_theta_delta ) const;
    swerve_output<is_field_relative> hat() const;
    swerve_output<is_field_relative> times( const double strafe_k, const double omega_k ) const;
    frc::ChassisSpeeds times( const units::meters_per_second_t strafe_k, const units::radians_per_second_t omega_k ) const;
    swerve_output<is_field_relative> limit( const double max_magnitude ) const;
    swerve_output<is_field_relative> limit( const double max_magnitude, const double max_theta ) const;
    bool is_stopped( const double strafe_deadband, const double omega_deadband ) const;
    void clamp( double turn_bias );
};


template<bool is_field_relative>
swerve_output<is_field_relative> operator+( const swerve_output<is_field_relative> &v1, const swerve_output<is_field_relative> &v2 );

template<bool is_field_relative>
swerve_output<is_field_relative> operator+( const swerve_output<is_field_relative> &v1, const vec2 &v2 );

template<bool is_field_relative>
swerve_output<is_field_relative> operator-( const swerve_output<is_field_relative> &v1, const swerve_output<is_field_relative> &v2 );

typedef swerve_output<true> field_relative_output;
typedef swerve_output<false> chassis_relative_output;

template<bool is_field_relative>
swerve_output<is_field_relative> swerve_output<is_field_relative>::deadbanded( double xy_deadband, double theta_deadband ) const {
    return swerve_output<is_field_relative>{
        .xy_output = this->xy_output.deadbanded( xy_deadband, 1.0 ),
        .omega_output = deadbanded( this->omega_output, theta_deadband, 1.0 )
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> swerve_output<is_field_relative>::move_towards( const vec2 &target, const double max_delta ) const {
    return swerve_output<is_field_relative>{
        .xy_output = this->xy_output.moved_towards( target, max_delta ),
        .omega_output = this->omega_output
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> swerve_output<is_field_relative>::move_towards( const swerve_output &target, const double max_xy_delta, const double max_theta_delta ) const {
    return swerve_output<is_field_relative>{
        .xy_output = this->xy_output.moved_towards( target.xy_output, max_xy_delta ),
        .omega_output = move_toward( this->omega_output, target.omega_output, max_theta_delta )
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> swerve_output<is_field_relative>::hat() const {
    return swerve_output<is_field_relative>{
        .xy_output = this->xy_output.hat(),
        .omega_output = this->omega_output
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> swerve_output<is_field_relative>::times( const double strafe_k, const double omega_k ) const {
    return swerve_output<is_field_relative>{
        .xy_output = this->xy_output * strafe_k,
        .omega_output = this->omega_output * omega_k
    };
}

template<bool is_field_relative>
frc::ChassisSpeeds swerve_output<is_field_relative>::times( const units::meters_per_second_t strafe_k, const units::radians_per_second_t omega_k ) const {
    return frc::ChassisSpeeds{
        xy_output.x * strafe_k,
        xy_output.y * strafe_k,
        omega_output * omega_k
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> swerve_output<is_field_relative>::limit( const double max_magnitude ) const {
    return swerve_output<is_field_relative>{
        .xy_output = this->xy_output.clamped( max_magnitude ),
        .omega_output = this->omega_output
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> swerve_output<is_field_relative>::limit( const double max_magnitude, const double max_theta ) const {
    return swerve_output<is_field_relative>{
        .xy_output = this->xy_output.clamped( max_magnitude ),
        .omega_output = std::clamp( this->omega_output, -max_theta, max_theta )
    };
}

template<bool is_field_relative>
bool swerve_output<is_field_relative>::is_stopped( const double strafe_deadband, const double omega_deadband ) const {
    return ( xy_output.length_squared() < ( strafe_deadband * strafe_deadband ) ) && ( std::abs( omega_output ) < omega_deadband );
}


template<bool is_field_relative>
swerve_output<is_field_relative> operator+( const swerve_output<is_field_relative> &v1, const swerve_output<is_field_relative> &v2 ) {
    return swerve_output<is_field_relative>{
        .xy_output = v1.xy_output + v2.xy_output,
        .omega_output = v1.omega_output + v2.omega_output
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> operator+( const swerve_output<is_field_relative> &v1, const vec2 &v2 ) {
    return swerve_output<is_field_relative>{
        .xy_output = v1.xy_output + v2,
        .omega_output = v1.omega_output
    };
}

template<bool is_field_relative>
swerve_output<is_field_relative> operator-( const swerve_output<is_field_relative> &v1, const swerve_output<is_field_relative> &v2 ) {
    return swerve_output<is_field_relative>{
        .xy_output = v1.xy_output - v2.xy_output,
        .omega_output = v1.omega_output - v2.omega_output
    };
}

template<bool is_field_relative>
void swerve_output<is_field_relative>::clamp( double turn_bias ) {
    // ensure valid bias
    turn_bias = std::clamp( turn_bias, 0.0, 1.0 );

    // first clamp strafing and turning individually, so we have each achievable on their own.
    xy_output = xy_output.clamped( 1 );
    omega_output = std::clamp( omega_output, -1.0, 1.0 );

    // fun stuff starts here
    double strafe_k = xy_output.length(); // 0...1
    double turn_k = std::abs( omega_output ); // 0...1
    double k = (strafe_k + turn_k); // 0...2
    constexpr double max_k = 1.0;
    double saturation = k - max_k;

    if( saturation < 0 ) {
        return;
    }

    // saturating speeds, need to compensate

    if( turn_k <= turn_bias ) {
        double adjusted_strafe_k = max_k - turn_k;
        xy_output *= adjusted_strafe_k / strafe_k;
        return;
    }

    if( strafe_k <= ( 1.0 - turn_bias ) ) {
        double adjusted_turn_k = max_k - strafe_k;
        omega_output *= adjusted_turn_k / turn_k;
        return;
    }

    // limit strafing and turning differently based on bias.
    double max_strafe_k = max_k - turn_bias;
    double max_turn_k = turn_bias;

    if( strafe_k != 0 ) {
        xy_output *= max_strafe_k / strafe_k;
    }
    if( turn_k != 0 ) {
        omega_output *= max_turn_k / turn_k;
    }
}
