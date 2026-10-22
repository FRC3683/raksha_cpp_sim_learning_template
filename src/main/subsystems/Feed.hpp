#pragma once

#include "config.hpp"

#include <ctre/phoenix6/CANdle.hpp>

#include "swerve.hpp"

#include "dave/morse_code_performer.hpp"
#include "shooter.hpp"

namespace Feed {
    enum class STATE {
        UNJAM,
        FEED,
        IDLE
    }; 

    static STATE curstate = STATE::UNJAM;
    TalonFX neck { 16, cfg::canivore};
    TalonFX spindexer { 14, cfg::canivore};

    frc::Timer unjam_timer; 

    void init() {
        unjam_timer.Restart();
        neck.GetConfigurator().Apply( ctre::phoenix6::configs::MotorOutputConfigs{}.WithInverted( ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive ) );
    }

    void handle_states () {
        if( shooter::wants_balls() ) {
            curstate = STATE::FEED;
        }
        neck.Set(1);
#if 0
        using enum STATE; 
        switch (curstate) {
            case UNJAM: {
                DutyCycleOut slow_unjam_control {-0.5};
                neck.SetControl (slow_unjam_control);
                spindexer.SetControl (slow_unjam_control);
                if (unjam_timer.HasElapsed(400_ms)) {
                    curstate = IDLE;
                }
            } break;

            case FEED:{
                VelocityTorqueCurrentFOC neck_control {5800_rpm * 0.75};
                DutyCycleOut spindexer_control {67};
                spindexer.SetControl(spindexer_control);
                // neck.SetControl(neck_control);
                neck.Set(1);

                if( !shooter::wants_balls() ) {
                    curstate = UNJAM;
                    unjam_timer.Restart();
                }
            } break;

            case IDLE: {
                DutyCycleOut stop {0};
                neck.SetControl(stop);
                spindexer.SetControl(stop);
            } break;
        }
#endif
    }

    STATE get_curstate() {
        return curstate; 
    }

    ctre::phoenix6::sim::TalonFXSimState& get_neck_sim() { return neck.GetSimState(); }


};