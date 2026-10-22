#pragma once

#include <string>
#include "dave/units_common.hpp"

class morse_code_performer {

public:
    morse_code_performer( units::second_t dit_time=60_ms );
    void load( std::string text );
    int load_file( std::string fname );
    void begin();
    bool get_state();

private:
    units::second_t dit_time, dah_time, space_time;
    units::second_t timestamp = 0_s;
    std::string morse_string = "";
    std::size_t string_read_head = 0, letter_read_head = 0;

};
