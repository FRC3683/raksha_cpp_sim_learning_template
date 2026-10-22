#include "dave/swerve/swerve_lowlevel.hpp"

#include "dave/torque_math.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

swerve_lowlevel::swerve_lowlevel( const swerve_cfg<4> &cfg )
  : ctre_swerve{ cfg.drivetrain_constants(), cfg.software_cfg.odom_freq(), cfg.module_constants( 0 ), cfg.module_constants( 1 ), cfg.module_constants( 2 ), cfg.module_constants( 3 ) },
    vis{ cfg.hardware_cfg.module_positions },
    antiskew{ cfg.tuning_cfg.antiskew },
    control_request{ dave_swerve_control_request{ *this } },
    frequency_hz{ cfg.software_cfg.odom_freq().value() },
    control_dts{ 1.0 / frequency_hz },
    strafe_freespeed{ cfg.hardware_cfg.free_speed() },
    omega_freespeed{ ( cfg.hardware_cfg.free_speed() / cfg.hardware_cfg.calc_chassis_radius() ).value() },
    coupling_kF{ cfg.tuning_cfg.coupling_kF },
    max_coupling_output{ cfg.tuning_cfg.max_coupling_output },
    peak_torque{ cfg.tuning_cfg.drive_current_limits.StatorCurrentLimit / cfg.hardware_cfg.drive_motor_stall_torque },
    peak_linear_accel{ cfg.hardware_cfg.peak_linear_accel },
    peak_angular_accel{ cfg.hardware_cfg.peak_angular_accel }
{
    if( frc::RobotBase::IsSimulation() ) {
        init_sim();
    }
    frc::SmartDashboard::PutNumber( "antiskew", cfg.tuning_cfg.antiskew );
}

void swerve_lowlevel::set_control( swerve_request req ) {
    control_request.req = req;
    SetControl( control_request );
}

void swerve_lowlevel::set_control_x_lock() {
    static ctre::phoenix6::swerve::requests::SwerveDriveBrake x_lock;
    SetControl( x_lock );
}

void swerve_lowlevel::set_control_disable() {
    static class disable : public ctre::phoenix6::swerve::requests::SwerveRequest {
    public:
        using SwerveModuleImpl = ctre::phoenix6::swerve::impl::SwerveModuleImpl;
        ctre::phoenix6::controls::StaticBrake brake;
        ctre::phoenix::StatusCode Apply( SwerveRequest::ControlParameters const &parameters, std::span<std::unique_ptr<SwerveModuleImpl> const> modulesToApply ) override {
            for (size_t i = 0; i < modulesToApply.size(); ++i) {
                modulesToApply[i]->Apply( std::move( brake ), std::move( brake ) );
            }
            return ctre::phoenix::StatusCode::OK;
        }
    } disable_req;
    SetControl( disable_req );
}

dave_swerve_control_request::dave_swerve_control_request( swerve_lowlevel &swervedrive )
  : swervedrive{ swervedrive } {}



ctre::phoenix::StatusCode dave_swerve_control_request::Apply( ControlParameters const &parameters, std::span<std::unique_ptr<SwerveModuleImpl> const> modules_to_apply ) {

    field_relative_output field_speeds = chassis_relative_output{ vec2{ parameters.currentChassisSpeed.vx.value(), parameters.currentChassisSpeed.vy.value() }, parameters.currentChassisSpeed.omega.value() }
        .to_field_relative( parameters.currentPose.Rotation().Radians() );

    chassis_relative_output chassis_relative = req.target_output.to_robot_relative( parameters.currentPose.Rotation().Radians() );

    // convert center of turning to be odometry center:
    chassis_relative.xy_output += req.center_of_rotation.rotated_by( -90_deg ) * chassis_relative.omega_output;

    // limit speeds
    chassis_relative.xy_output = chassis_relative.xy_output.clamped( 1.0 );
    chassis_relative.omega_output = std::clamp( chassis_relative.omega_output, -1.0, 1.0 );

    // bias + more limiting
    chassis_relative.clamp( req.turn_bias );

    double max_omega_torque = req.max_angular_accel / swervedrive.peak_angular_accel;
    double max_strafe_torque = req.max_linear_accel / swervedrive.peak_linear_accel;

    // field relative torque limiting
    field_relative_output target_field_output = chassis_relative.to_field_relative( parameters.currentPose.Rotation().Radians() );
    double omega_out = limit_torque( target_field_output.omega_output, field_speeds.omega_output, swervedrive.omega_freespeed.value(), swervedrive.peak_torque * max_omega_torque );
    vec2 strafe_out = limit_torque( target_field_output.xy_output, field_speeds.xy_output, swervedrive.strafe_freespeed.value(), swervedrive.peak_torque * max_strafe_torque, req.max_linear_accel, parameters.updatePeriod );
    chassis_relative = field_relative_output{ strafe_out, omega_out }.to_robot_relative( parameters.currentPose.Rotation().Radians() );

    // hysteresis to avoid jitter
    strafe_stopped.calculate( chassis_relative.xy_output.length() );
    turn_stopped.calculate( chassis_relative.omega_output );

    if( strafe_stopped ) {
        chassis_relative.xy_output = vec2::zero;
    }
    if( turn_stopped ) {
        chassis_relative.omega_output = 0;
    }

    // prevent wheel jitter
    if( strafe_stopped && turn_stopped ) {
        for( auto &module : modules_to_apply ) {
            module->Apply( NeutralOut{}, NeutralOut{} );
        }
        return ctre::phoenix::StatusCode::OK;
    }

    // anti skew
    double px = chassis_relative.xy_output.y;
    double py = -chassis_relative.xy_output.x;
    chassis_relative.xy_output.x += px * chassis_relative.omega_output * frc::SmartDashboard::GetNumber( "antiskew", swervedrive.antiskew );
    chassis_relative.xy_output.y += py * chassis_relative.omega_output * frc::SmartDashboard::GetNumber( "antiskew", swervedrive.antiskew );

    // convert to units for kinematics
    auto wheel_speeds = parameters.kinematics->ToSwerveModuleStates(
        frc::ChassisSpeeds::Discretize( chassis_relative.times( swervedrive.strafe_freespeed, swervedrive.omega_freespeed ), parameters.updatePeriod ) );

    swervedrive.applied_output = chassis_relative;

    int i = 0;
    for( auto &module : modules_to_apply ) {
        auto &output = wheel_speeds[i];

        auto current_azimuth = module->GetPosition( false ).angle;
        output.Optimize( current_azimuth );

        // scale drive speed inversely with azimuth error:
        auto azimuth_err = ( current_azimuth - output.angle ).Radians().value(); // Rotation2d subtraction auto wraps to -pi...pi
        double drive_scaling = 1 - int_pow( 2 * azimuth_err / PI, 4 );

        // coupling compensation:
        double predict_azimuth_omega = signum( azimuth_err ) * std::sqrt( 2 * std::abs( azimuth_err ) );
        auto coupling_ff = ( predict_azimuth_omega * swervedrive.coupling_kF );
        coupling_ff = units::volt_t{ std::clamp( coupling_ff.value(), -swervedrive.max_coupling_output.value(), swervedrive.max_coupling_output.value() ) };

        auto drive_output = drive_scaling * 12_V * output.speed / swervedrive.strafe_freespeed;

        module->Apply( std::move(
            drive_request.WithOutput( drive_output + coupling_ff ) 
        ), std::move(
            steer_request.WithPosition( output.angle.Radians() )
        ) );

        ++i;
    }

    return ctre::phoenix::StatusCode::OK;
}

void swerve_lowlevel::init_sim() {
    m_lastSimTime = ctre::phoenix6::utils::GetCurrentTime();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = ctre::phoenix6::utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* Use the measured time delta, get battery voltage from WPILib */
        UpdateSimState( deltaTime, frc::RobotController::GetBatteryVoltage() );
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void swerve_lowlevel::update_nt() {
    static bool first_run = true;
    if( first_run ) {
        first_run = false;
        frc::SmartDashboard::PutData( "field", &field2d );
    }
    auto drive_state = GetState();
    vis.update( drive_state.ModuleStates, drive_state.Pose.Rotation().Degrees() );
    field2d.SetRobotPose( drive_state.Pose );
}

frc::Pose2d swerve_lowlevel::get_pose() const {
    return GetState().Pose;
}

vec2 swerve_lowlevel::get_pose_vec( swerve_lowlevel::SwerveDriveState state ) {
    return vec2{ state.Pose.X().value(), state.Pose.Y().value() };
}

vec2 swerve_lowlevel::get_pose_vec() const {
    return get_pose_vec( GetState() );
}

chassis_relative_output swerve_lowlevel::get_applied_output() const {
    return applied_output;
}

field_relative_output swerve_lowlevel::get_field_relative_applied_output( SwerveDriveState state ) const {
    auto chassis_out = get_applied_output();
    return chassis_out.to_field_relative( state.Pose.Rotation().Radians() );
}

field_relative_output swerve_lowlevel::get_field_relative_applied_output() const {
    return get_field_relative_applied_output( GetState() );
}

chassis_relative_output swerve_lowlevel::get_chassis_speeds( swerve_lowlevel::SwerveDriveState state ) {
    return chassis_relative_output{ vec2{ state.Speeds.vx.value(), state.Speeds.vy.value() }, state.Speeds.omega.value() };
}

chassis_relative_output swerve_lowlevel::get_chassis_speeds() const {
    return get_chassis_speeds( GetState() );
}

field_relative_output swerve_lowlevel::get_field_speeds( swerve_lowlevel::SwerveDriveState state ) {
    return get_chassis_speeds( state ).to_field_relative( state.Pose.Rotation().Radians() );
}

field_relative_output swerve_lowlevel::get_field_speeds() const {
    return get_field_speeds( GetState() );
}
