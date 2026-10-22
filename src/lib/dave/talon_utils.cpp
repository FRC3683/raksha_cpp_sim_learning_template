#include "dave/talon_utils.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

void configure( ctre::phoenix6::hardware::TalonFX &talon, ctre::phoenix6::configs::TalonFXConfiguration &cfg, int attempts ) {
    if( !frc::SmartDashboard::ContainsKey( "health/ctre_configurations" ) ) {
        frc::SmartDashboard::PutBoolean( "health/ctre_configurations", true );
    }
    for( int i = 0; i < attempts; ++i ) {
        if( talon.GetConfigurator().Apply( cfg ).IsOK() ) {
            return;
        }
    }
    frc::SmartDashboard::PutBoolean( "health/ctre_configurations", false );
    // TODO spit warning?
}
