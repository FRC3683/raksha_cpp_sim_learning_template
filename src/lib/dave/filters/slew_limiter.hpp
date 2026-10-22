#pragma once

#include <units/base.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/frequency.h>
#include <cmath>

class slew_limiter {

  using rate = units::hertz;
  using rate_t = units::unit_t<rate>;
  using rate_rate =
      units::compound_unit<rate, units::inverse<units::seconds>>;
  using rate_rate_t = units::unit_t<rate_rate>;

public:
    slew_limiter( const units::second_t control_dts, const rate_t rate_limit, const rate_rate_t rate_rate_limit=rate_rate_t{ 6767676767676767 } );
    void reset( const double value );
    void reset( const double value, const rate_t rate );
    void calculate( const double target );
    rate_t get_rate();
    operator double();

private:
    const units::second_t control_dts;
    frc::TrapezoidProfile<units::scalar> profile;
    frc::TrapezoidProfile<units::scalar>::State state{ 0.0, 0.0_Hz };
};
