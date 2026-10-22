#pragma once

#include <functional>
#include <chrono>
#include <array>

#include <units/length.h>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix/StatusCodes.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Notifier.h>
#include <frc/TimedRobot.h>

#include "dave/swerve/swerve_config.hpp"
#include "dave/swerve/swerve_output.hpp"
#include "dave/swerve/swerve_vis.hpp"
#include "dave/pose2d.hpp"
#include "dave/swerve/swerve_request.hpp"
#include "dave/filters/hysteresis.hpp"

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <ctre/phoenix6/swerve/SwerveModule.hpp>

#include <stdio.h>

class swerve_lowlevel;

class dave_swerve_control_request : public ctre::phoenix6::swerve::requests::SwerveRequest {
    using SwerveModuleImpl = ctre::phoenix6::swerve::impl::SwerveModuleImpl;
    using NeutralOut = ctre::phoenix6::controls::NeutralOut;
    using StaticBrake = ctre::phoenix6::controls::StaticBrake;
    using ModuleRequest = SwerveModuleImpl::ModuleRequest;
    using PositionVoltage = ctre::phoenix6::controls::PositionVoltage;
    using VoltageOut = ctre::phoenix6::controls::VoltageOut;

public:
    dave_swerve_control_request( swerve_lowlevel &swervedrive );
    ctre::phoenix::StatusCode Apply( ControlParameters const &parameters, std::span<std::unique_ptr<SwerveModuleImpl> const> modules_to_apply ) override;

private:
    swerve_lowlevel &swervedrive;
    swerve_request req;

    PositionVoltage steer_request = PositionVoltage{ 0_deg }
        .WithEnableFOC( true )
        .WithUpdateFreqHz( 0_Hz );
    VoltageOut drive_request = VoltageOut{ 0_V }
        .WithEnableFOC( true )
        .WithUpdateFreqHz( 0_Hz );

    ModuleRequest module_req = ModuleRequest{}
        .WithDriveRequest( ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage )
        .WithSteerRequest( ctre::phoenix6::swerve::impl::SteerRequestType::Position )
        .WithEnableFOC( true );

    hysteresis strafe_stopped{ 0.00025, 0.0005 };
    hysteresis turn_stopped{ 0.00025, 0.0005 };

    friend class swerve_lowlevel;
};


typedef ctre::phoenix6::swerve::SwerveDrivetrain<ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::TalonFX, ctre::phoenix6::hardware::CANcoder> ctre_swerve;

class swerve_lowlevel : public ctre_swerve {

    swerve_vis<4> vis;
    frc::Field2d field2d{};

    const double antiskew;
    units::radian_t formX[4];

    static constexpr units::second_t kSimLoopPeriod = 1_ms;
    std::unique_ptr<frc::Notifier> m_simNotifier;
    units::second_t m_lastSimTime;

    dave_swerve_control_request control_request;

    chassis_relative_output applied_output;

public:
    const double frequency_hz, control_dts;
    const units::meters_per_second_t strafe_freespeed;
    const units::radians_per_second_t omega_freespeed;
    const units::volt_t coupling_kF, max_coupling_output;
    const double peak_torque;
    const units::meters_per_second_squared_t peak_linear_accel;
    const units::radians_per_second_squared_t peak_angular_accel;

public:
    swerve_lowlevel( const swerve_cfg<4> &cfg );
    void set_control( swerve_request req );
    void set_control_x_lock();
    void set_control_disable();
    void init_sim();
    void update_nt();
    frc::Pose2d get_pose() const;
    vec2 get_pose_vec() const;
    chassis_relative_output get_applied_output() const;
    field_relative_output get_field_relative_applied_output( swerve_lowlevel::SwerveDriveState state ) const;
    field_relative_output get_field_relative_applied_output() const;
    chassis_relative_output get_chassis_speeds() const;
    field_relative_output get_field_speeds() const;

    static vec2 get_pose_vec( swerve_lowlevel::SwerveDriveState state );
    static chassis_relative_output get_chassis_speeds( swerve_lowlevel::SwerveDriveState state );
    static field_relative_output get_field_speeds( swerve_lowlevel::SwerveDriveState state );

    friend class dave_swerve_control_request;
};

