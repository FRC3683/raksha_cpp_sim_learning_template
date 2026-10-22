#pragma once

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <fstream>

#include <dave/juice.hpp>

#include "subsystems/swerve.hpp"
#include "constants.hpp"

/**
 * cool robot stats like:
 *  - odometer
 *  - uptime
 */

namespace stats_tracking {

    static std::string fpath = "/path/to/stats"; // TODO

    static uint64_t auton_uptime_us = 0, tele_uptime_us = 0;
    static uint64_t prev_tick = 0;
    static double distance_travelled_m = 0;
    static double total_watt_hours = 0;
    static bool saved = true;

    void stats_init() {
        if( frc::RobotBase::IsSimulation() ) {
            return;
        }

        std::ifstream infile{ fpath, std::ios::binary };
        if( !infile.is_open() ) {
            return;
        }

        infile >> auton_uptime_us;
        infile >> tele_uptime_us;
        infile >> distance_travelled_m;
        infile >> total_watt_hours;

        infile.close();
    }

    static void stats_save() {
        if( frc::RobotBase::IsSimulation() ) {
            return;
        }

        std::ofstream outfile{ fpath, std::ios::binary };
        if( !outfile.is_open() ) {
            return;
        }

        outfile << auton_uptime_us;
        outfile << tele_uptime_us;
        outfile << distance_travelled_m;
        outfile << total_watt_hours + juice::watthours_consumed().value();

        outfile.close();
    }

    void stats_update() {
        uint64_t now = frc::RobotController::GetFPGATime();
        uint64_t delta_us = now - prev_tick;
        if( now < prev_tick ) {
            // FPGA rolled over, just pretend its 100us delta and dont worry about it.
            delta_us = 100;
        }
        units::second_t delta_seconds = units::microsecond_t{ (double)delta_us };

        if( frc::DriverStation::IsEnabled() ) {
            saved = false;
        }
        if( frc::DriverStation::IsDisabled() && !saved ) {
            stats_save();
            saved = true;
        }

        double distance = swerve::get_field_speeds().xy_output.length() * delta_seconds.value();
        distance_travelled_m += distance;

        if( frc::DriverStation::IsAutonomousEnabled() ) {
            auton_uptime_us += delta_us;
        } else if( frc::DriverStation::IsTeleopEnabled() ) {
            tele_uptime_us += delta_us;
        }

        frc::SmartDashboard::PutNumber( "stats/odometer_kms", distance_travelled_m / 1000.0 );
        frc::SmartDashboard::PutNumber( "stats/auton_uptime", auton_uptime_us / 1000000 );
        frc::SmartDashboard::PutNumber( "stats/teleop_uptime", tele_uptime_us / 1000000 );
        frc::SmartDashboard::PutNumber( "stats/total_watthours", total_watt_hours + juice::watthours_consumed().value() );

        prev_tick = now;
    }

}
