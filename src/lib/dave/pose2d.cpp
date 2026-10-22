#include "dave/pose2d.hpp"

pose2d::pose2d( vec2 xy, units::radian_t heading )
  : xy{ xy }, heading{ heading }
{}

pose2d::pose2d( frc::Pose2d pose )
  : xy{ pose.Translation() }, heading{ pose.Rotation().Radians() }
{}
