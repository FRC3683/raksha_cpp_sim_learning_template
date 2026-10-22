#include "dave/swerve/swerve_output.hpp"

template<>
chassis_relative_output chassis_relative_output::to_robot_relative( units::radian_t robot_heading ) const {
    return *this;
}

template<>
chassis_relative_output field_relative_output::to_robot_relative( units::radian_t robot_heading ) const {
    return chassis_relative_output{
        .xy_output = this->xy_output.rotated_by( -robot_heading ),
        .omega_output = this->omega_output
    };
}

template<>
field_relative_output field_relative_output::to_field_relative( units::radian_t robot_heading ) const {
    return *this;
}

template<>
field_relative_output chassis_relative_output::to_field_relative( units::radian_t robot_heading ) const {
    return field_relative_output{
        .xy_output = this->xy_output.rotated_by( robot_heading ),
        .omega_output = this->omega_output
    };
}
