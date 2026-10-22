#pragma once

#include <dave/ctre_common.hpp>
#include <dave/units_common.hpp>

#include <dave/preferences.hpp>
#include <dave/filters/debouncer.hpp>
#include "config.hpp"
#include "constants.hpp"

namespace intake {

    enum class intention {
        STOW,
        DEPLOY_IDLE,
        SUCC,
        VOMIT,
        JOSTLE,
        PERIODIC_JOSTLE,
    };

    enum class state {
        ZEROING,
        STOW,
        DEPLOYING_HOPPER,
        DEPLOY_IDLE,
        SUCC,
        VOMIT,
        SPRING,
        PERIODIC_SPRING_UP,
        PERIODIC_SPRING_DOWN,
    };

    // HARDWARE
    static TalonFX pivot{ cfg::can_intake_pivot, cfg::canivore };
    static TalonFX roller_master{ cfg::can_intake_roller_master, cfg::canivore };
    static TalonFX roller_slave{ cfg::can_intake_roller_slave, cfg::canivore };

    // STATUS SIGNALS
    static auto &roller_vel = roller_master.GetVelocity( false );
    static auto &roller_current = roller_master.GetStatorCurrent( false );
    static auto &roller_voltage = roller_master.GetMotorVoltage( false );
    static auto &pivot_pos = pivot.GetPosition( false );
    static auto &pivot_vel = pivot.GetVelocity( false );
    static auto &pivot_current = pivot.GetStatorCurrent( false );

    // CONTROL REQUESTS
    static auto zero_request = DutyCycleOut{ -0.3 }
        .WithEnableFOC( true )
        .WithIgnoreSoftwareLimits( true );
    static auto arm_position_request = DynamicMotionMagicVoltage{ 0_deg, 0_deg_per_s, 8_tr_per_s_sq }
        .WithSlot( 0 )
        .WithEnableFOC( true );
    static auto arm_spring_req = PositionTorqueCurrentFOC{ k_intake_spring_center }
        .WithSlot( 1 );
    static auto roller_openloop = VoltageOut{ 0_V }
        .WithEnableFOC( true );
    static auto roller_jostle_req = TorqueCurrentFOC{ 30_A }.WithMaxAbsDutyCycle( 0.3 );

    static auto roller_follow = Follower{ roller_master.GetDeviceID(), MotorAlignmentValue::Aligned };

    // FILTERS
    static debouncer is_stalling = debouncer::rising_edge( 300_ms );

    // STATE
    static bool intake_enabled = true;
    static intention intake_intent = intention::STOW;
    static state intake_current_state = state::ZEROING;
    static state intake_prev_state = state::STOW;
    static frc::Timer state_timer{};
    static dash_toggle dash_toggle_enable{ "intake_enable" };

    void init() {
        configure( pivot, cfg::intake::pivot_config );
        configure( roller_master, cfg::intake::roller_config );
        configure( roller_slave, cfg::intake::roller_config );

        ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 250_Hz, roller_current, roller_voltage );
        ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 100_Hz, roller_vel, pivot_pos, pivot_vel, pivot_current );
    }

    void read_inputs() {
        ctre::phoenix6::BaseStatusSignal::RefreshAll( roller_vel, roller_current, roller_voltage, pivot_pos, pivot_vel, pivot_current );
    }

    void handle_states() {
        using enum state;

        if( intake_current_state != intake_prev_state ) {
            state_timer.Restart();
        }

        state desired_state = STOW;
        switch( intake_intent ) {
            case intention::STOW:              desired_state = STOW;                 break;
            case intention::DEPLOY_IDLE:       desired_state = DEPLOY_IDLE;          break;
            case intention::SUCC:              desired_state = SUCC;                 break;
            case intention::VOMIT:             desired_state = VOMIT;                break;
            case intention::JOSTLE:            desired_state = SPRING;               break;
            case intention::PERIODIC_JOSTLE:   desired_state = PERIODIC_SPRING_UP;   break;
        }

        switch( intake_current_state ) {
            case ZEROING: {
                is_stalling.calculate( units::math::abs( pivot_vel.GetValue() ) < 1_deg_per_s && units::math::abs( pivot_current.GetValue() ) > 40_A );
                if( is_stalling ) {
                    pivot.SetPosition( k_intake_deployed - 5_deg );
                    intake_current_state = desired_state;
                }
                break;
            }
            case PERIODIC_SPRING_UP: {
                if( desired_state != PERIODIC_SPRING_UP ) {
                    intake_current_state = desired_state;
                }
                if( state_timer.Get() > 1.5_s ) {
                    intake_current_state = PERIODIC_SPRING_DOWN;
                }
                break;
            }
            case DEPLOYING_HOPPER: {
                if( pivot_pos.GetValue() < k_intake_deployed + 10_deg ) {
                    intake_current_state = desired_state;
                }
                break;
            }
            case PERIODIC_SPRING_DOWN: {
                if( pivot_pos.GetValue() < k_intake_deployed + 10_deg || desired_state != PERIODIC_SPRING_UP ) {
                    intake_current_state = desired_state;
                }
                break;
            }
            default: {
                if( intake_intent != intention::STOW && pivot_pos.GetValue() > 80_deg ) {
                    intake_current_state = DEPLOYING_HOPPER;
                } else {
                    intake_current_state = desired_state;
                }
                break;
            }
        }

        intake_prev_state = intake_current_state;
    }

    static void set_pivot_angle( units::degree_t angle, bool slow=false ) {
        pivot.SetControl( arm_position_request.WithPosition( 0_deg ).WithVelocity( slow ? 0_deg_per_s : 0_deg_per_s ) );
    }

    static void set_roller_speed( double speed ) {
        roller_master.SetControl( roller_openloop.WithOutput( 12_V * speed ) );
    }

    void set_outputs() {
        using enum state;

        if( !intake_enabled || !dash_toggle_enable ) {
            pivot.StopMotor();
            roller_master.StopMotor();
            roller_slave.StopMotor();
            return;
        }

        switch( intake_current_state ) {
            case ZEROING:
                pivot.SetControl( zero_request );
                set_roller_speed( k_intake_roller_push_hopper_speed );
                break;
            case STOW:
                set_pivot_angle( k_intake_stowed );
                roller_master.StopMotor();
                break;
            case DEPLOYING_HOPPER:
                set_pivot_angle( k_intake_deployed, true );
                set_roller_speed( k_intake_roller_push_hopper_speed );
                break;
            case DEPLOY_IDLE:
                set_pivot_angle( k_intake_deployed );
                roller_master.StopMotor();
                break;
            case SUCC:
                set_pivot_angle( k_intake_deployed );
                set_roller_speed( k_intake_succ_speed );
                break;
            case VOMIT:
                set_pivot_angle( k_intake_deployed );
                set_roller_speed( k_intake_vomit_speed );
                break;
            case SPRING:
            case PERIODIC_SPRING_UP:
                pivot.SetControl( arm_spring_req );
                roller_master.SetControl( roller_jostle_req );
                break;
            case PERIODIC_SPRING_DOWN:
                set_pivot_angle( k_intake_deployed );
                set_roller_speed( k_intake_roller_push_hopper_speed );
                break;
        }
        roller_slave.SetControl( roller_follow );
    }


    void disable() {
        intake_enabled = false;
    }
    void enable() {
        intake_enabled = true;
    }

    void intend( intention intent ) {
        using enum intention;

        intake_intent = intent;
        switch( intent ) {
            case JOSTLE:
            default:
                break;
        }
    }

    void hardstop_zero() {
        pivot.SetPosition( k_intake_hardstop );
    }

    void auto_zero() {
        intake_current_state = state::ZEROING;
        is_stalling.calculate( false ); // reset
    }

    state intake_get_state() {
        return intake_current_state;
    }

}
