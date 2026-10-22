#include "dave/filters/ema_filter.hpp"

ema_filter::ema_filter( double alpha, double initial_value )
  : alpha{ alpha }, value{ initial_value }
{}

void ema_filter::calculate( double input ) {
    value = ( alpha * input ) + ( (1.0 - alpha) * value );
}

void ema_filter::reset( double value ) {
    this->value = value;
}

ema_filter::operator double() const {
    return value;
}
