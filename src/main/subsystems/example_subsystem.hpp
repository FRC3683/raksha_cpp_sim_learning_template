#pragma once

#include <dave/ctre_common.hpp>
#include <dave/units_common.hpp>

#include "config.hpp"
#include "constants.hpp"

namespace example_subsystem {

    enum class intention {
        GOAL1,
        GOAL2
    };

    enum class state {
        STATE1,
        STATE2,
        STATE3,
        STATE4
    };

    // HARDWARE
    static TalonFX motor{ can_id, canbus };

    // STATUS SIGNALS
    static auto &motor_speed = motor.GetVelocity( false );

    // CONTROL REQUESTS
    static auto openloop = VoltageOut{ 0_V }
        .WithEnableFOC( true );
    static auto position_request = PositionVoltage{ 0_deg };

    // STATE
    intention current_intention;
    state current_state;
    static bool subsystem_enabled = true;

    void init() {
        configure( motor, configuration );

        ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 100_Hz, motor_speed );
    }

    void read_inputs() {
        ctre::phoenix6::BaseStatusSignal::RefreshAll( motor_speed );
    }

    void handle_states() {
        using enum state;

        switch( current_state ) {
            case STATE1: {
                
            } break;

            case STATE2: {
                
            } break;

            case STATE3: {
                
            } break;

            case STATE4: {
                
            } break;
        }
    }

    void set_outputs() {
        using enum state;

        if( !subsystem_enabled ) {
            motor.Set( 0 );
            return;
        }

        switch( current_state ) {
            case STATE1: {
                
            } break;

            case STATE2: {
                
            } break;

            case STATE3: {
                
            } break;

            case STATE4: {
                
            } break;
        }
    }


    void disable() {
        subsystem_enabled = false;
    }
    void enable() {
        subsystem_enabled = true;
    }

}
