#include "dave/oi.hpp"


namespace oi {

    frc::XboxController xbox_driver{ 0 };
    frc::GenericHID xkeys{ 1 };

    static double custom_shaping( double d ) {
        if( d < 0 ) {
            return -custom_shaping( -d );
        }
        static const double l = 0.3;
        static auto g = []( const double x ){
            return x * x / ( 2 * l );
        };
        static auto h = []( const double x ){
            if( x < l ) {
                return x;
            }
            return g( x ) + x - g( l );
        };
        static const double h_1 = h(1);

        return h( std::clamp( d, 0.0, 1.0 ) ) / h_1;
    }

    double shape_input( const double d, input_shaping_function shape ) {

        switch( shape ) {
            case input_shaping_function::SQUARE:
                return d * d * signum( d );
            case input_shaping_function::CUBE:
                return d * d * d;
            case input_shaping_function::CUSTOM:
                return custom_shaping( d );
            case input_shaping_function::LINEAR:
            default:
                return d;
        }
        return d;
    }

    double get_axis_driver( int axis, double deadband, input_shaping_function shape ) {
        return shape_input( deadbanded( xbox_driver.GetRawAxis( axis ), deadband, 1.0 ), shape );
    }

    vec2 get_left_stick_driver( double deadband, input_shaping_function shape ) {
        vec2 v = vec2{ -xbox_driver.GetLeftY(), -xbox_driver.GetLeftX() }.clamped( 1.0 ).deadbanded( deadband, 1.0 );
        double l = shape_input( v.length(), shape );
        return vec2{ l, v.theta() };
    }

    vec2 get_right_stick_driver( double deadband, input_shaping_function shape ) {
        vec2 v = vec2{ -xbox_driver.GetRightY(), -xbox_driver.GetRightX() }.clamped( 1.0 ).deadbanded( deadband, 1.0 );
        double l = shape_input( v.length(), shape );
        return vec2{ l, v.theta() };
    }

    bool get_driver_button( int button ) {
        return xbox_driver.GetRawButton( button );
    }

    template<int pov>
    static bool get_driver_pov() {
        return xbox_driver.GetPOV() == pov;
    }

    bool driver_dpad_up() {
        return xbox_driver.GetPOV() == 0;
    }

    bool driver_dpad_right() {
        return xbox_driver.GetPOV() == 90;
    }

    bool driver_dpad_down() {
        return xbox_driver.GetPOV() == 180;
    }

    bool driver_dpad_left() {
        return xbox_driver.GetPOV() == 270;
    }

    bool driver_any_input() {
        for( int btn = frc::XboxController::Button::kA; btn <= frc::XboxController::Button::kRightStick; ++btn ) {
            if( get_driver_button( btn ) ) {
                return true;
            }
        }
        for( int axis = frc::XboxController::Axis::kLeftX; axis <= frc::XboxController::Axis::kRightY; ++axis ) {
            if( get_axis_driver( axis ) != 0 ) {
                return true;
            }
        }
        if( xbox_driver.GetPOV() != -1 ) {
            return true;
        }
        return false;
    }

}
