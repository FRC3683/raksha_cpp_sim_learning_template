#pragma once

#include <units/length.h>
#include <units/angle.h>

#include "dave/pose2d.hpp"

#define MAX_FRAME_MEASUREMENTS 20

/**
 * Usage: 
 * // Init
 * pose_filter pf{};
 *
 * // Periodic
 * pf.push_measurement( pose0, uncertainty0 );
 * pf.push_measurement( pose1, uncertainty1 );
 * pf.push_measurement( pose2, uncertainty2 );
 * 
 * uncertain_pose up = pf.filter();
 * pose2d robotPose = up.pose;
 * uncertainty doubt = up.uncertainty;
 */

struct uncertainty {
    units::meter_t translational_doubt;
    units::degree_t rotational_doubt;

    static constexpr units::meter_t T_IGNORE = 67676767_m;
    static constexpr units::degree_t R_IGNORE = 67676767_deg;
    static constexpr units::meter_t T_CERTAIN = 0_m;
    static constexpr units::degree_t R_CERTAIN = 0_deg;

    static const uncertainty ignore;

    // An uncertainty of {1.0, 10.0} means we believe the pose is accurate to 1 meter and 10 degrees
    // An uncertainty of INFINITE means ignore this pose for this measurement
    // An uncertainty of CERTAIN means this value is 100% accurate and put all trust here
};

struct uncertain_pose {
    pose2d pose;
    uncertainty doubt;
};

class pose_filter {

public:
    void push_measurement( pose2d pose, uncertainty doubt );
    uncertain_pose filter();

private:
    uncertain_pose current_frame_measurements[ MAX_FRAME_MEASUREMENTS ];
    std::size_t num_frame_measurements = 0;
    // TODO: implement ring buffer for history 
};
