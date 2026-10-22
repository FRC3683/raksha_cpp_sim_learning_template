#pragma once

#include <functional>
#include <units/angle.h>

#include "dave/vector.hpp"
#include "dave/pose2d.hpp"


namespace field {

    enum symmetry_type {
        MIRROR_X,
        ROTATIONAL
    };

    extern symmetry_type field_symmetry;
    extern vec2 field_size;

    vec2 mirror_x( vec2 coords );
    vec2 mirror_y( vec2 coords );
    vec2 rotate( vec2 coords );

    units::radian_t mirror_x( units::radian_t heading );
    units::radian_t rotate( units::radian_t heading );

    pose2d mirror_x( pose2d coords );
    pose2d rotate( pose2d coords );

    vec2 flip( vec2 coords );
    units::radian_t flip( units::radian_t heading );
    pose2d flip( pose2d heading );

    vec2 auto_flip( vec2 red_coords );
    units::radian_t auto_flip( units::radian_t red_heading );
    pose2d auto_flip( pose2d red_heading );

}

struct red_vec2 {
    vec2 red_coords;

    red_vec2( double x, double y );
    red_vec2( vec2 vec );
    operator vec2() const;
};

struct blue_vec2 {
    vec2 blue_coords;

    blue_vec2( double x, double y );
    blue_vec2( vec2 vec );
    operator vec2() const;
};

struct red_heading {
    units::radian_t red_angle;

    operator units::radian_t() const;
};

struct blue_heading {
    units::radian_t blue_angle;

    operator units::radian_t() const;
};

struct red_pose2d {
    pose2d red_coords;

    red_pose2d( vec2 red_vec, units::radian_t red_angle );
    operator pose2d() const;
};

struct blue_pose2d {
    pose2d blue_coords;

    blue_pose2d( vec2 blue_vec, units::radian_t blue_angle );
    operator pose2d() const;
};
