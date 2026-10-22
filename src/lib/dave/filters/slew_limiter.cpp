#include "dave/filters/slew_limiter.hpp"

#include "dave/math.hpp"

slew_limiter::slew_limiter( const units::second_t control_dts, const rate_t rate_limit, const rate_rate_t rate_rate_limit )
  : control_dts{ control_dts },
    profile{ frc::TrapezoidProfile<units::scalar>::Constraints{ rate_limit, rate_rate_limit } }
{}

void slew_limiter::reset( const double value ) {
    state.position = value;
    state.velocity = 0_Hz;
}

void slew_limiter::reset( const double value, const rate_t rate ) {
    state.position = value;
    state.velocity = rate;
}

void slew_limiter::calculate( const double target ) {
    state = profile.Calculate( control_dts, state, frc::TrapezoidProfile<units::scalar>::State{ target, 0_Hz } );
}

slew_limiter::operator double() {
    return state.position.value();
}

slew_limiter::rate_t slew_limiter::get_rate() {
    return state.velocity;
}
