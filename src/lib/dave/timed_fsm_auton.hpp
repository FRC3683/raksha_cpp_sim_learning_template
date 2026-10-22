#pragma once

#include <optional>
#include <frc/Timer.h>

#include "dave/auto_selector.hpp"
#include "dave/schedulable.hpp"
#include "dave/field.hpp"

constexpr units::second_t auton_end_time = 20_s;

struct timed_fsm_auton : public schedulable {

    units::second_t get_auton_time();
    units::second_t get_state_time();
    bool has_passed( units::second_t time );
    bool has_state_passed( units::second_t time );
    bool has_left( units::second_t time );

    int get_state();
    int get_prev_state();
    void set_state( int s );
    void increment_state();

    virtual void handle_state( int current_state ) = 0;
    virtual std::string dashboard_name() = 0;
    virtual std::optional<red_pose2d> preferred_starting_pose();

    void enter() override;
    void periodic() override;

    operator auton();

private:
    int state = 0, prev_state = -1;
    frc::Timer auton_timer{}, state_timer{};
};