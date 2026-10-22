#include "dave/preferences.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

preference::preference( std::string key, double default_value )
  : key{ "driver_prefs/" + key }, default_value{ default_value }
{}

preference::operator double() {
#ifdef COMPETITION
    return default_value;
#else
    if( !initialized && !frc::SmartDashboard::ContainsKey( this->key ) ) {
        initialized = true;
        frc::SmartDashboard::PutNumber( this->key, default_value );
    }
    return frc::SmartDashboard::GetNumber( key, default_value );
#endif
}


dash_toggle::dash_toggle( std::string key, bool default_value )
  : key{ "dashboard_toggles/" + key }, default_value{ default_value }
{}

dash_toggle::operator bool() {
    if( !initialized && !frc::SmartDashboard::ContainsKey( this->key ) ) {
        initialized = true;
        frc::SmartDashboard::PutBoolean( this->key, default_value );
    }
    return frc::SmartDashboard::GetBoolean( key, default_value );
}
