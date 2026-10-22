#pragma once

#include "dave/schedulable.hpp"
#include "dave/timed_fsm_auton.hpp"
#include "subsystems/swerve.hpp"
#include "dave/auto_selector.hpp"
#include "dave/field.hpp"

struct : public timed_fsm_auton {
    std::optional<red_pose2d> preferred_starting_pose() override {
        return red_pose2d{ vec2{ 3, 4 }, 0_deg };
    }

    std::string dashboard_name() override {
        return "first auton";
    }

    void handle_state( int state ) override {
        switch( state ) {
            case 0:
                swerve::pid_line( red_vec2{3, 3}, red_vec2{-3, 1}, swerve::pid_to_point_params{}, swerve::strafe_constraints{ .max_speed=3_mps } );
                swerve::snap_towards( red_vec2{1, 1} );
                if( swerve::is_at_point() ) {
                    increment_state();
                }
                break;
            case 1:
                swerve::pid_arc( red_vec2{6, 7}, red_vec2{0.466, 4.877}, swerve::pid_to_point_params{}, swerve::strafe_constraints{ .max_speed=3_mps } );
                if( has_state_passed( 0.5_s ) ) {
                    swerve::snap_heading( red_heading{ 45_deg } );
                } else {
                    swerve::turn_idle();
                }
                break;
        }
    }
} auton1;
