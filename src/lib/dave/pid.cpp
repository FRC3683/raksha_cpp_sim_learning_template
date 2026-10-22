#include "dave/pid.hpp"

#include <frc/MathUtil.h>
#include "dave/math.hpp"

pid::pid( double control_dts )
  : dts{ control_dts }
{}

void pid::apply( const pid_config &cfg ) {
    slots[0] = cfg.slot0;
    slots[1] = cfg.slot1;
    slots[2] = cfg.slot2;
    slots[3] = cfg.slot3;
    continuous = cfg.continuous;
    min_input = cfg.min_input;
    max_input = cfg.max_input;
    input_range = max_input - min_input;
    half_input_range = input_range / 2.0;
    saturated_output = cfg.saturated_output;
}

void pid::InitSendable( wpi::SendableBuilder& builder ) {
    builder.SetSmartDashboardType("pid controller");
    for( int i = 0; i < NUM_SLOTS; ++i ) {
        std::string base = "slot_" + std::to_string( i ) + "/";
        builder.AddDoubleProperty( base + "kGain",
            [this, i] { return slots[i].kGain; },
            [this, i]( double gain ) { slots[i].kGain = gain; }
        );
        builder.AddDoubleProperty( base + "kI",
            [this, i] { return slots[i].kI; },
            [this, i]( double kI ) { slots[i].kI = kI; }
        );
        builder.AddDoubleProperty( base + "kD",
            [this, i] { return slots[i].kD; },
            [this, i]( double kD ) { slots[i].kD = kD; }
        );
        builder.AddDoubleProperty( base + "kV",
            [this, i] { return slots[i].kV; },
            [this, i]( double kV ) { slots[i].kV = kV; }
        );
        builder.AddDoubleProperty( base + "kA",
            [this, i] { return slots[i].kA; },
            [this, i]( double kA ) { slots[i].kA = kA; }
        );
    }
}

void pid::reset() {
    prev_err = 0;
    err_integral = 0;
    has_measurement = false;
}

double pid::calculate( double err, gain_slot slot ) {
    slot_configs &gains = slots[(int)slot];

    has_measurement = true;

    double err_derivative;

    if( continuous ) {
        
        err = frc::InputModulus( err, min_input, max_input );
        err_derivative = frc::InputModulus( err - prev_err, -half_input_range, half_input_range ) / dts;
    } else {
        err_derivative = ( err - prev_err ) / dts;
    }

    prev_err = err;

    err_integral += err * dts;
    err_integral = std::clamp( err_integral, -saturated_output / gains.kI, saturated_output / gains.kI );

    return ( gains.kGain * err ) + ( gains.kGain * gains.kI * err_integral ) + ( gains.kGain * gains.kD * err_derivative );
}

double pid::calculate( double measurement, double setpoint, gain_slot slot ) {
    return calculate( setpoint - measurement, slot );
}

double pid::calculate( double measurement, const pid_target &setpoint, gain_slot slot ) {
    double feedforward = ( setpoint.velocity * slots[slot].kV ) + ( setpoint.acceleration * slots[slot].kA );
    return calculate( measurement, setpoint.position, slot ) + feedforward;
}

bool pid::within( double tolerance ) {
    return has_measurement && std::abs( prev_err ) <= tolerance;
}

// profiled_pid::profiled_pid( const profiled_pid_config &cfg, double control_dts )
//   : pid{ cfg, control_dts },
//     kV{ cfg.kV }, kA{ cfg.kA },
//     max_velocity{ cfg.max_velocity },
//     target_acceleration{ cfg.target_acceleration }
// {}

// void profiled_pid::reset( double measured_position, double measured_velocity ) {

// }

// double profiled_pid::calculate( double measurement, double setpoint ) {
//     return 0;
// }

double clamp_output( double output, double speed_percent, double max_speed_percent, double headroom_kP ) {
    double headroom = ( signum( output ) * max_speed_percent ) - speed_percent;
    if( headroom < 0 ) {
        headroom = 0;
    }
    double output_limit = max_speed_percent + ( headroom * headroom_kP );
    return std::clamp( output, -output_limit, output_limit );
}
