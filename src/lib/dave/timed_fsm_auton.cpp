#include "dave/timed_fsm_auton.hpp"

units::second_t timed_fsm_auton::get_auton_time() {
    return auton_timer.Get();
}

units::second_t timed_fsm_auton::get_state_time() {
    return state_timer.Get();
}

bool timed_fsm_auton::has_passed( units::second_t time ) {
    return get_auton_time() > time;
}

bool timed_fsm_auton::has_state_passed( units::second_t time ) {
    return get_state_time() > time;
}

bool timed_fsm_auton::has_left( units::second_t time ) {
    return auton_end_time - get_auton_time() > time;
}

int timed_fsm_auton::get_state() {
    return state;
}

int timed_fsm_auton::get_prev_state() {
    return prev_state;
}

void timed_fsm_auton::set_state( int s ) {
    state_timer.Restart();
    prev_state = state;
    state = s;
}

void timed_fsm_auton::increment_state() {
    set_state( state + 1 );
}

std::optional<red_pose2d> timed_fsm_auton::preferred_starting_pose() {
    return std::nullopt;
}

void timed_fsm_auton::enter() {
    state = 0;
    prev_state = -1;
    auton_timer.Restart();
    state_timer.Restart();
}

void timed_fsm_auton::periodic() {
    handle_state( state );
}

timed_fsm_auton::operator auton() {
    return auton{ dashboard_name(), this, preferred_starting_pose() };
}
