#pragma once

#include <units/length.h>
#include <frc/geometry/Pose2d.h>
#include "dave/vector.hpp"

struct pose2d {
    vec2 xy;
    units::radian_t heading;

    template<units::length_unit unit>
    frc::Pose2d in() const {
        return frc::Pose2d{ unit{ xy.x }, unit{ xy.y }, frc::Rotation2d{ heading } };
    }

    pose2d( vec2 xy=vec2::zero, units::radian_t heading=0_deg );
    pose2d( frc::Pose2d pose );
};
