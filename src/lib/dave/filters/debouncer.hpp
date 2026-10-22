#pragma once

#include <units/time.h>

class debouncer {
public:
    debouncer( units::second_t rise_time, units::second_t fall_time, bool initial_value=false );
    static debouncer rising_edge( units::second_t rise_time, bool initial_value=false );
    static debouncer falling_edge( units::second_t fall_time, bool initial_value=false );
    static debouncer symmetric( units::second_t trip_time, bool initial_value=false );

    void calculate( bool value );
    operator bool() const;

private:
    units::second_t rise_time, fall_time;
    units::second_t timestamp = -1_s;
    bool value;
};
