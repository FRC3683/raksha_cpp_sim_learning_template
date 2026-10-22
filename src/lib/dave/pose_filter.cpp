#include "dave/pose_filter.hpp"

const uncertainty uncertainty::ignore{ uncertainty::T_IGNORE, uncertainty::R_IGNORE };

void pose_filter::push_measurement( pose2d pose, uncertainty doubt ) {
    if( num_frame_measurements >= MAX_FRAME_MEASUREMENTS ) {
        // TODO print warning
        return;
    }
    current_frame_measurements[ num_frame_measurements++ ] = uncertain_pose{ pose, doubt };
}

uncertain_pose pose_filter::filter() {
    vec2 xy = vec2::zero;
    units::degree_t deg = 0_deg;

    uncertainty robot_uncertainty{ uncertainty::T_IGNORE, uncertainty::R_IGNORE };

    int min_translational_uncertainty_idx = -1;
    int min_rotational_uncertainty_idx = -1;
    // Calculate Robot Uncertainty as minimum of all uncertainties
    for( std::size_t i = 0; i < num_frame_measurements; ++i ) {
        units::meter_t curr_tran = current_frame_measurements[i].doubt.translational_doubt;
        units::degree_t curr_rot = current_frame_measurements[i].doubt.rotational_doubt;

        if(curr_tran < robot_uncertainty.translational_doubt) {
            robot_uncertainty.translational_doubt = curr_tran;
            min_translational_uncertainty_idx = i;
        }

        if(curr_rot < robot_uncertainty.rotational_doubt) {
            robot_uncertainty.rotational_doubt = curr_rot;
            min_rotational_uncertainty_idx = i;
        }
    }

    // Translational pose filtering
    if( robot_uncertainty.translational_doubt == uncertainty::T_CERTAIN ) {
        xy = current_frame_measurements[ min_translational_uncertainty_idx ].pose.xy;
    } else {
        double weighted_x = 0.0;
        double weighted_y = 0.0;
        double uncertainty_sum = 0.0;

        for( std::size_t i = 0; i < num_frame_measurements; ++i ) {

            units::meter_t translational_uncertainty = current_frame_measurements[i].doubt.translational_doubt;
            vec2 curr = current_frame_measurements[i].pose.xy;

            if( translational_uncertainty != uncertainty::T_IGNORE ) {
                double scale = 1.0 / ( translational_uncertainty.value() * translational_uncertainty.value() );

                weighted_x += scale * curr.x;
                weighted_y += scale * curr.y;
                uncertainty_sum += scale;
            }

            xy = { weighted_x / uncertainty_sum, weighted_y / uncertainty_sum };
        }
    }

    // Rotational pose filtering
    if( robot_uncertainty.rotational_doubt == uncertainty::R_CERTAIN ) {
        deg = current_frame_measurements[ min_rotational_uncertainty_idx ].pose.heading;
    } else {
        units::degree_t weighted_deg = 0_deg;
        double uncertainty_sum = 0.0;

        for( std::size_t i = 0; i < num_frame_measurements; ++i ) {

            units::degree_t rotational_uncertainty = current_frame_measurements[i].doubt.rotational_doubt;
            units::degree_t curr_deg = current_frame_measurements[i].pose.heading;

            if( rotational_uncertainty != uncertainty::R_IGNORE ) {
                double scale = 1.0 / ( rotational_uncertainty.value() * rotational_uncertainty.value() );

                weighted_deg += scale * curr_deg;
                uncertainty_sum += scale;
            }

            deg = weighted_deg / uncertainty_sum;
        }
    }

    num_frame_measurements = 0;
    return uncertain_pose( pose2d{ xy, deg }, robot_uncertainty );
}
