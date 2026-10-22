#pragma once

#include <cstdio>
#include <cstdlib>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "dave/swerve/swerve_vis.hpp"
#include "dave/vector.hpp"
#include "dave/math.hpp"

template<unsigned int num_modules>
class swerve_vis {
    frc::Mechanism2d mechanism;
    frc::MechanismRoot2d *root;
    frc::MechanismLigament2d *chassis;
    frc::MechanismLigament2d *ligaments[num_modules];

public:
    swerve_vis( const vec2 module_positions[num_modules], const char *dashboard_key="swerve_vis" );
    void update( std::vector<frc::SwerveModuleState> module_states, units::degree_t heading );
};

static const double dim = 60;

template<unsigned int num_modules>
swerve_vis<num_modules>::swerve_vis( const vec2 module_positions[num_modules], const char *dashboard_key )
      : mechanism{ dim, dim }
{
    if( num_modules > 9 ) {
        FRC_ReportError( frc::err::Error, "really?" );
        std::exit( EXIT_FAILURE );
    }
    double radius_sq = 0;
    for( unsigned int i = 0; i < num_modules; ++i ) {
        double length_sq = module_positions[i].length_squared();
        if( length_sq > radius_sq ) {
            radius_sq = length_sq;
        }
    }
    double radius = std::sqrt( radius_sq );

    root = mechanism.GetRoot( "root", dim * 0.5, dim * 0.5 );
    chassis = root->Append<frc::MechanismLigament2d>( "chassis", 0, 0_deg );

    for( unsigned int i = 0; i < num_modules; ++i ) {
        char buffer[3] = "m ";
        buffer[1] = i + '0';
        const vec2 &pos = module_positions[i];
        frc::MechanismLigament2d *m = chassis->Append<frc::MechanismLigament2d>( buffer, 20 * pos.length() / radius, pos.theta(), 0 );
        frc::MechanismLigament2d *m2 = m->Append<frc::MechanismLigament2d>( "lol", 0, -pos.theta(), 0 );
        ligaments[i] = m2->Append<frc::MechanismLigament2d>( "limb", 6, 0_deg );
        frc::MechanismLigament2d *l = ligaments[i]; // this fixes some obscure templating error lmao
        l->Append<frc::MechanismLigament2d>( "arrow1", 4, 135_deg, 3 );
        l->Append<frc::MechanismLigament2d>( "arrow2", 4, -135_deg, 3 );
    }

    frc::SmartDashboard::PutData( dashboard_key, &mechanism );
}

template<unsigned int num_modules>
void swerve_vis<num_modules>::update( std::vector<frc::SwerveModuleState> module_states, units::degree_t heading ) {
    chassis->SetAngle( heading );
    for( unsigned int i = 0; i < num_modules; ++i ) {
        frc::MechanismLigament2d *l = ligaments[i];
        frc::SwerveModuleState speed = module_states[i];
        l->SetLength( 2 + speed.speed.value() * 2 );
        l->SetAngle( speed.angle.Degrees() );
    }
}
