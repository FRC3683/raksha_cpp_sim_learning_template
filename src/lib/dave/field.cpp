#include "dave/field.hpp"

#include <frc/DriverStation.h>


namespace field {

    symmetry_type field_symmetry = symmetry_type::ROTATIONAL;
    vec2 field_size = vec2{ 16.541, 8.036 };

    vec2 mirror_x( vec2 coords ) {
        return vec2{ field_size.x - coords.x, coords.y };
    }
    vec2 mirror_y( vec2 coords ) {
        return vec2{ coords.x, field_size.y - coords.y };
    }
    vec2 rotate( vec2 coords ) {
        return field_size - coords;
    }

    units::radian_t mirror_x( units::radian_t heading ) {
        return 180_deg - heading;
    }
    units::radian_t rotate( units::radian_t heading ) {
        return 180_deg + heading;
    }

    pose2d mirror_x( pose2d coords ) {
        return pose2d{ mirror_x( coords.xy ), mirror_x( coords.heading ) };
    }
    pose2d rotate( pose2d coords ) {
        return pose2d{ rotate( coords.xy ), rotate( coords.heading ) };
    }

    vec2 flip( vec2 coords ) {
        switch( field_symmetry ) {
        case MIRROR_X:
            return mirror_x( coords );
        case ROTATIONAL:
            return rotate( coords );
        default:
            return coords;
        }
    }

    units::radian_t flip( units::radian_t heading ) {
        switch( field_symmetry ) {
            case MIRROR_X:
                return mirror_x( heading );
            case ROTATIONAL:
                return rotate( heading );
            default:
                return heading;
        }
    }

    pose2d flip( pose2d coords ) {
        switch( field_symmetry ) {
            case MIRROR_X:
                return mirror_x( coords );
            case ROTATIONAL:
                return rotate( coords );
            default:
                return coords;
        }
    }

    vec2 auto_flip( vec2 red_coords ) {
        bool is_red = frc::DriverStation::GetAlliance().value_or( frc::DriverStation::Alliance::kRed ) == frc::DriverStation::Alliance::kRed;
        return is_red ? red_coords : flip( red_coords );
    }

    units::radian_t auto_flip( units::radian_t red_heading ) {
        bool is_red = frc::DriverStation::GetAlliance().value_or( frc::DriverStation::Alliance::kRed ) == frc::DriverStation::Alliance::kRed;
        return is_red ? red_heading : flip( red_heading );
    }

    pose2d auto_flip( pose2d red_coords ) {
        bool is_red = frc::DriverStation::GetAlliance().value_or( frc::DriverStation::Alliance::kRed ) == frc::DriverStation::Alliance::kRed;
        return is_red ? red_coords : flip( red_coords );
    }

}

red_vec2::red_vec2( double x, double y ) : red_coords{ x, y } {}
red_vec2::red_vec2( vec2 vec ) : red_coords{ vec } {}
red_vec2::operator vec2() const {
    return field::auto_flip( red_coords );
}

blue_vec2::blue_vec2( double x, double y ) : blue_coords{ x, y } {}
blue_vec2::blue_vec2( vec2 vec ) : blue_coords{ vec } {}
blue_vec2::operator vec2() const {
    return field::auto_flip( field::flip( blue_coords ) );
}

red_heading::operator units::radian_t() const {
    return field::auto_flip( red_angle );
}

blue_heading::operator units::radian_t() const {
    return field::auto_flip( field::flip( blue_angle ) );
}


red_pose2d::red_pose2d( vec2 red_vec, units::radian_t red_angle )
: red_coords{ red_vec, red_angle } {}

red_pose2d::operator pose2d() const {
    return field::auto_flip( red_coords );
}



blue_pose2d::blue_pose2d( vec2 blue_vec, units::radian_t blue_angle )
: blue_coords{ blue_vec, blue_angle } {}

blue_pose2d::operator pose2d() const {
    return field::auto_flip( field::flip( blue_coords ) );
}
