#pragma once

#include <frc/PowerDistribution.h>
#include <units/energy.h>


namespace juice {

    void update( frc::PowerDistribution &pd );
    units::watt_hour_t watthours_consumed();
    double juice_remaining();

}
