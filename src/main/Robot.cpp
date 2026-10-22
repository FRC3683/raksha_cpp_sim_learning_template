// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/Watchdog.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "dave/auto_selector.hpp"
#include "dave/juice.hpp"

#include "config.hpp"
#include "constants.hpp"
#include "autos.hpp"
#include "stats_tracking.hpp"

#include "subsystems/swerve.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/vision.hpp"
#include "subsystems/leds.hpp"
#include <frc/event/BooleanEvent.h>
#include "subsystems/shooter.hpp"
#include "magic_enum/magic_enum.hpp"
#include "subsystems/Feed.hpp"
#include "subsystems/Samintake.hpp"

#include "sim.hpp"

#include <iostream>

using namespace frc;

#define PRAC_BOT_SRL_NUMBER ""

static auto_selector selector{ auton1 };
static schedulable *auto_routine = nullptr;

frc::PowerDistribution PDH{ 1, frc::PowerDistribution::ModuleType::kRev };

//shooter variables

    double snap_angle_deg = 0.0;
    double k_turn_deadband = 0.5;

static void robot_init() {
    shooter::init();
    shooter::zero_turn_pose();

    bool is_comp_bot = frc::RobotController::GetSerialNumber() != PRAC_BOT_SRL_NUMBER || frc::RobotBase::IsSimulation();
    frc::SmartDashboard::PutBoolean( "health/is_comp", is_comp_bot );
    if( !is_comp_bot ) {
        cfg::prac_bot_configs();
    }

    field::field_symmetry = field::symmetry_type::ROTATIONAL;

    swerve::init();
    intake::init();
    intake::disable();
    leds::init();
    shooter::zero_turn_pose(); 
    Feed::init();

    stats_tracking::stats_init();

    if( frc::RobotBase::IsSimulation() ) {
        get_owl_sim = shooter::get_owl_sim;
        get_hood_sim = shooter::get_hood_sim;
        get_flywheel_sim = shooter::get_flywheel_sim;
        get_pivot_sim = Samintake::get_pivot_sim;
        get_neck_sim = Feed::get_neck_sim;
        sim_init();
    }
}


static void read_inputs() {
    juice::update( PDH );
    vision::read_inputs();
    swerve::read_inputs();
    intake::read_inputs();
    // shooter::zero_pose();
    SmartDashboard::PutNumber("curpose", shooter::calculate_crt_angle());
    SmartDashboard::PutNumber("sixteen", shooter::get_sixteen_encoder_value());
    SmartDashboard::PutNumber("seventeen", shooter::get_seventeen_encoder_value());

}


static void teleop_periodic() {
    SmartDashboard::PutString("curr state", magic_enum::enum_name(shooter::curstate)); 
    SmartDashboard::PutNumber("cur pose of shooter", shooter::hood_motor.GetPosition().GetValueAsDouble());
    if( auto_routine ) {
        if( oi::driver_any_input() ) {
            auto_routine->cancel();
        }
        auto_routine->process();
    }

    if (oi::xbox_driver.GetBackButton()) {
        shooter::curstate = shooter::state::ATRACK; 
    }

    if (oi::xbox_driver.GetBackButtonReleased()){
        shooter::curstate = shooter::state::ASHOOT;
    }

    
    if( oi::get_left_stick_driver().length_squared() > 0.001 ) {
        swerve::strafe_tele();
    }


    else if( oi::get_right_stick_driver().length_squared() > 0.001 ) {
        swerve::turn_tele();
    }

    if (oi::xbox_driver.GetXButton()) {
        vec2 zerovec = vec2 {0, 0};
        swerve::reset_translation(zerovec);
    }


    if (oi::xbox_driver.GetYButton()) {
        Samintake::set_intention(Samintake::Intention::succc);
    }

    if (oi::xbox_driver.GetBButton()) {
        Samintake::set_intention(Samintake::Intention::idle);

    }

    if (oi::xbox_driver.GetAButton()) {
        Samintake::set_intention(Samintake::Intention::vomit);
    }

}

static void autonomous_periodic() {
    if( auto_routine ) {
        auto_routine->process();
    }
}

static void test_periodic() {}

static void disabled_periodic() {
    selector.disabled_periodic();
    bool close_enough = true;
    if( selector.get_selected().preferred_starting_pose.has_value() ) {
        pose2d preferred = selector.get_selected().preferred_starting_pose.value();
        vec2 xy_diff = preferred.xy - swerve::get_vec2();
        units::degree_t heading_diff = wrap_angle( preferred.heading - swerve::get_heading() );
        close_enough = xy_diff.length() < 0.1 && units::math::abs( heading_diff ) < 5_deg;
    }
    frc::SmartDashboard::PutBoolean( "health/auto_lineup", close_enough );

}

static void handle_subsystems() {
    swerve::handle_states();
    intake::handle_states();
    //shooter must be before feed bc thats how feed knows to want ballz
    shooter::handle_states();
    Feed::handle_states(); 
}

static void set_outputs() {
    swerve::set_outputs();
    intake::set_outputs();

    leds::set_outputs();

    swerve::update_dashboard();
}


class robot : public frc::TimedRobot {

public:
    robot() : frc::TimedRobot{ units::second_t{k_control_dts} } {
        robot_init();
    }

    void RobotPeriodic() final override {
        read_inputs();
        if( frc::DriverStation::IsTeleopEnabled() ) {
            teleop_periodic();
        } else if( frc::DriverStation::IsAutonomousEnabled() ) {
            autonomous_periodic();
        } else if( frc::DriverStation::IsTestEnabled() ) {
            test_periodic();
        } else if( frc::DriverStation::IsDisabled() ) {
            disabled_periodic();
        }

        if( oi::xbox_driver.GetPOV() == 180 ) {
            swerve::swerve_hardware().ResetRotation( frc::Rotation2d{ (units::radian_t) blue_heading{ 0_rad } } );
        }

        handle_subsystems();

        set_outputs();

        stats_tracking::stats_update();

        if( frc::RobotBase::IsSimulation() ) {
            sim_periodic();
        }
    }

    void AutonomousInit() final override {
        auto_routine = selector.get_selected().to_schedule;
        if( auto_routine ) {
            auto_routine->schedule();
        }
    }
    void AutonomousExit() final override  {
        // if( auto_routine ) {
        //     auto_routine->cancel();
        // }
    }

    void TeleopInit() final override {
            shooter::set_to_ASHOOT();
            SmartDashboard::PutNumber("cur intention of shooter", int(shooter::curstate));

    }
    void TeleopExit() final override {}

    void DisabledInit() final override {
        swerve::strafe_idle();
        swerve::turn_idle();
    }
    void DisabledExit() final override {}

    void TestInit() final override {
        swerve::disable();
        shooter::set_to_ZEROING();
        SmartDashboard::PutNumber("cur intention of shooter", int(shooter::curstate));
    }
    void TestExit() final override {
        swerve::enable();
    }

    void SimulationInit() final override {}
    void SimulationPeriodic() final override {}
};


#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<robot>();
}
#endif
