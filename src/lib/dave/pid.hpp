#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

struct slot_configs {
    double kGain = 0.0, kI = 0.0, kD = 0.0;
    double kV = 0.0, kA = 0.0;
    double profile_max_accel = 0.0, profile_cruise_velocity = 0.0;
};

enum gain_slot : unsigned char {
    SLOT_0, SLOT_1, SLOT_2, SLOT_3
};

constexpr unsigned char NUM_SLOTS = 4;

struct pid_config {
    slot_configs slot0{}, slot1{}, slot2{}, slot3{};
    bool continuous = false;
    double min_input = 0.0, max_input = 0.0;
    double saturated_output = 1.0;
    char *nt_string = nullptr;
};

struct pid_target {
    double position = 0.0, velocity = 0.0, acceleration = 0.0;
};

class pid : public wpi::Sendable {
    const double dts;
    slot_configs slots[NUM_SLOTS] = { 0 };
    bool continuous = false;
    double min_input = 0;
    double max_input = 0;
    double input_range = 0;
    double half_input_range = 0;
    double saturated_output = 0;

    double prev_err = 0;
    double err_integral = 0;
    bool has_measurement = false;

public:
    pid( double control_dts=0.02 );

    void apply( const pid_config &cfg );

    void InitSendable( wpi::SendableBuilder& builder ) override;

    void reset();
    double calculate( double err, gain_slot slot=SLOT_0 );
    double calculate( double measurement, double setpoint, gain_slot slot=SLOT_0 );
    double calculate( double measurement, const pid_target &setpoint, gain_slot slot=SLOT_0 );
    bool within( double tolerance );
};

class profiled_pid : public pid {

public:


    void reset( double position, double velocity );
    double calculate( double measurement, double setpoint, double target_velocity, double target_acceleration );
};

double clamp_output( double output, double speed_percent, double max_speed_percent, double headroom_kP );
