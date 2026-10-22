#include "dave/auto_selector.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Errors.h>

#include "dave/oi.hpp"

auto_selector::auto_selector( std::initializer_list<auton> autons ) {
    if( autons.size() > banksize * 2 ) {
        FRC_ReportError( frc::err::Error, "too many autos!" );
    }
    int i = 0;
    for( auto &a : autons ) {
        this->autons[i] = a;
        ++i;
        if( i == banksize * 2 ) {
            break;
        }
    }
    select( 0 );
}

void auto_selector::disabled_periodic() {
    bool shift = oi::xkeys.GetRawButton( shift_btn );
    for( int i = 0; i < banksize; ++i ) {
        if( oi::xkeys.GetRawButton( select_buttons[i] ) ) {
            select( i + ( shift ? banksize : 0 ) );
            break;
        }
    }
}

auton& auto_selector::get_selected() {
    return autons[selected];
}

void auto_selector::select( int i ) {
    selected = i;
    frc::SmartDashboard::PutString( "auton", get_selected().dashboard_name );
}
