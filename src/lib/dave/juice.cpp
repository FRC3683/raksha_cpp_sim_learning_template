#include "dave/juice.hpp"

#include <units/current.h>
#include <units/voltage.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>


namespace juice {

    static bool initialized = false;

    static units::watt_hour_t energy_used = 0_Wh;
    static units::second_t prev_tick = -1_s;
    static units::volt_t initial_voltage = -1_V;

    void update( frc::PowerDistribution &pd ) {
        auto now = frc::Timer::GetFPGATimestamp();
        auto delta = now - prev_tick;
        prev_tick = now;

        if( !initialized ) {
            initialized = true;
            initial_voltage = units::volt_t{ pd.GetVoltage() };
            return;
        }

        auto current = units::ampere_t{ pd.GetTotalCurrent() };
        auto voltage = units::volt_t{ pd.GetVoltage() };
        auto power = current * voltage;
        auto energy = power * delta;

        energy_used += energy;

        frc::SmartDashboard::PutNumber( "juice/initial_voltage", initial_voltage.value() );
        frc::SmartDashboard::PutNumber( "juice/energy_consumed", energy_used.value() );
        frc::SmartDashboard::PutNumber( "juice/voltage", voltage.value() );
        frc::SmartDashboard::PutNumber( "juice/current", current.value() );
    }

    units::watt_hour_t watthours_consumed() {
        return energy_used;
    }

    double juice_remaining() {
        return 1; // TODO
    }

}
