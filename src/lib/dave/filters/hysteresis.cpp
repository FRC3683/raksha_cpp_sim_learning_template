#include "dave/filters/hysteresis.hpp"

#include <cmath>

hysteresis::hysteresis ( double tight_threshold, double loose_threshold, bool initial_value )
  : tight_threshold{ tight_threshold },
    loose_threshold{ loose_threshold },
    value{ initial_value }
{}

void hysteresis::calculate( double measurement ) {
    if( value ) {
        value = std::abs( measurement ) < loose_threshold;
    } else {
        value = std::abs( measurement ) < tight_threshold;
    }
}

hysteresis::operator bool() const {
    return value;
}
