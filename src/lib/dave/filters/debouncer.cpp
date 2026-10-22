#include "dave/filters/debouncer.hpp"

#include <frc/Timer.h>

debouncer::debouncer( units::second_t rise_time, units::second_t fall_time, bool initial_value )
  : rise_time{ rise_time }, fall_time{ fall_time }, value{ initial_value }
{}

debouncer debouncer::rising_edge( units::second_t rise_time, bool initial_value ) {
    return debouncer{ rise_time, 0_s, initial_value };
}

debouncer debouncer::falling_edge( units::second_t fall_time, bool initial_value ) {
    return debouncer{ 0_s, fall_time, initial_value };
}

debouncer debouncer::symmetric( units::second_t trip_time, bool initial_value ) {
    return debouncer{ trip_time, trip_time, initial_value };
}

void debouncer::calculate( bool value ) {
    if( timestamp == -1_s ) {
        timestamp = frc::Timer::GetFPGATimestamp();
    }

    if( this->value == value ) {
        timestamp = frc::Timer::GetFPGATimestamp();
        return;
    }

    units::second_t trip_time = this->value ? fall_time : rise_time;

    if( trip_time <= 1_ms || frc::Timer::GetFPGATimestamp() - timestamp >= trip_time ) {
        this->value = value;
        timestamp = frc::Timer::GetFPGATimestamp();
    }
}

debouncer::operator bool() const {
    return value;
}
