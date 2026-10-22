#pragma once

#include <frc/XboxController.h>
#include "dave/vector.hpp"
#include "dave/math.hpp"


namespace oi {

#define DEFAULT_DEADBAND 0.075

    enum class input_shaping_function {
        LINEAR,
        SQUARE,
        CUBE,
        CUSTOM
    };

    extern frc::XboxController xbox_driver;
    extern frc::GenericHID xkeys;

    double shape_input( double d, input_shaping_function shape );

    double get_axis_driver( int axis, double deadband=DEFAULT_DEADBAND, input_shaping_function shape=input_shaping_function::LINEAR );

    vec2 get_left_stick_driver( double deadband=DEFAULT_DEADBAND, input_shaping_function shape=input_shaping_function::LINEAR );
    vec2 get_right_stick_driver( double deadband=DEFAULT_DEADBAND, input_shaping_function shape=input_shaping_function::LINEAR );

    bool get_driver_button( int button );


    bool driver_dpad_up();
    bool driver_dpad_right();
    bool driver_dpad_down();
    bool driver_dpad_left();

    bool driver_any_input();

}
