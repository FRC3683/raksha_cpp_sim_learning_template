#pragma once

#include <string>

#include <frc/RobotBase.h>
#include <dave/pose_filter.hpp>

#include "LimelightHelpers.h"

#define NUM_CAMERAS 3

namespace vision {

    std::string camera_names[] = { "limelight-zero", "limelight-one", "limelight-two" };

    uncertain_pose measurements[ NUM_CAMERAS ];

    static void sim_read_inputs() {
        // TODO
    }

    // call this before drivetrain read inputs
    void read_inputs() {
        if( frc::RobotBase::IsSimulation() ) {
            sim_read_inputs();
            return;
        }

        // TODO tell limelight to COOL DOWN in disabled pls :)

        for( std::size_t camera_index = 0; camera_index < NUM_CAMERAS; ++camera_index ) {
            auto estimate = LimelightHelpers::getBotPoseEstimate_wpiBlue( camera_names[ camera_index ] );
            bool ignore = estimate.tagCount < 2;
            // TODO add more conditions here
            if( ignore ) {
                measurements[ camera_index ].doubt = uncertainty::ignore;
                continue;
            }
            measurements[ camera_index ] = {
                .pose = estimate.pose,
                .doubt = {
                    .translational_doubt = 1_m, // TODO
                    .rotational_doubt = 1_deg
                }
            };
        }
    }

    uncertain_pose get_raw_pose( int camera_index ) {
        return measurements[ camera_index ];
    }

    uncertain_pose get_filtered_pose() {
        // TODO
        return uncertain_pose{
            .pose = pose2d{ vec2::zero, 0_deg },
            .doubt = uncertainty::ignore
        };
    }

}
