#pragma once

#include <dave/units_common.hpp>
#include <dave/swerve/swerve_lowlevel.hpp>
#include <dave/oi.hpp>
#include <dave/pid.hpp>
#include <dave/math.hpp>
#include <dave/preferences.hpp>
#include <frc/Timer.h>

#include "config.hpp"
#include "constants.hpp"
#include <frc/RobotController.h>

namespace swerve {
    enum class strafe_mode {
        X_LOCK, // turn mode also needs to be in x_lock to put the chassis in x_lock
        IDLE, // openloop with 0 output
        OPENLOOP, // open...loop
        JOYSTICK, // default teleop control
        PID_TO_POINT, // thanks, 2056
        PID_LINE, // came to me in a dream
        PID_ARC, // thought of it on the bus
        PID_SPEEDS, // pid velocity on chassis speeds
        STTBOLLS, // some turning thing based on limelight stuff
    };

    enum class turn_mode {
        X_LOCK, // strafe mode also needs to be in x_lock to put the chassis in x_lock
        IDLE, // openloop with 0 output
        OPENLOOP, // open...loop
        JOYSTICK, // default teleop control
        SNAP, // peak, since 2023
        FACE_TARGET, // look at target coordinates
        PID_SPEED, // pid velocity on turn speed
        STTBOLLS, // some turning thing based on limelight stuff
        FACE_STRAFE_DIR, // thanks, 1114
    };

    struct strafe_constraints {
        units::meters_per_second_t max_speed = 4.5_mps;
        units::meters_per_second_squared_t max_accel = 25_mps_sq;
    };

    struct turn_constraints {
        units::radians_per_second_t max_speed = 600_deg_per_s;
        units::radians_per_second_squared_t max_accel = 900_deg_per_s_sq;
    };


    // HARDWARE
    static swerve_lowlevel *lowlevel_inst = nullptr;

    struct pid_to_point_params {
        double feedforward = 0.0;
        units::meter_t deadzone = 10_mm;
        gain_slot slot = SLOT_0;
    };
    struct heading_snap_params {
        units::radians_per_second_t feedforward_velocity = 0_deg_per_s;
        units::radian_t deadzone=1.0_deg;
        gain_slot slot = SLOT_0;
        double desaturation_bias = -0.15;
    };

    // request data
    static strafe_mode active_strafe_mode = strafe_mode::OPENLOOP;
    static turn_mode active_turn_mode = turn_mode::OPENLOOP;
    static strafe_constraints active_strafe_constraints;
    static turn_constraints active_turn_constraints;
    static vec2 xy_out;
    static double omega_out;
    static double strafe_desaturation_bias, turn_desaturation_bias;
    static vec2 pid_to_point_target, pid_line_direction, pid_arc_center;
    static pid_to_point_params active_pid_point_params;
    static heading_snap_params active_snap_params;
    static units::radian_t heading_snap_target;
    static vec2 snap_target;
    static units::radian_t snap_target_offset;
    static vec2 center_of_turning = vec2::zero;

    // state
    static bool swerve_enabled = true;
    static swerve_lowlevel::SwerveDriveState swerve_synced_state;
    static frc::Timer time_since_last_collision;

    // closed loop
    static pid heading_snap_pid{ k_control_dts };
    static pid to_point_pid{ k_control_dts };

    // driver preferences
    preference teleop_strafe_speed{ "drive_speed", 1.0 };
    preference teleop_turn_speed{ "turn_speed", 0.67 };
    preference teleop_torque{ "teleop_torque", 0.5 };



    swerve_lowlevel& swerve_hardware() {
        return *lowlevel_inst;
    }

    swerve_lowlevel::SwerveDriveState swerve_get_synced_state() {
        return swerve_synced_state;
    }



    void init() {
        if( lowlevel_inst ) {
            // TODO warn reinit?
            return;
        }

        time_since_last_collision.Start();
        lowlevel_inst = new swerve_lowlevel{ cfg::swerve::lowlevel_cfg };

        heading_snap_pid.apply( cfg::swerve::heading_snap_cfg );
        to_point_pid.apply( cfg::swerve::pid_to_point_cfg );

        frc::SmartDashboard::PutData( "tuning/heading snap", &heading_snap_pid );
        frc::SmartDashboard::PutData( "tuning/pid to point", &to_point_pid );

    }



    frc::Pose2d get_pose() {
        return swerve_synced_state.Pose;
    }

    vec2 get_vec2() {
        return vec2{ get_pose().Translation() };
    }

    units::radian_t get_heading() {
        return swerve_synced_state.Pose.Rotation().Radians();
    }

    chassis_relative_output get_chassis_speeds() {
        return chassis_relative_output{ vec2{ swerve_synced_state.Speeds.vx.value(), swerve_synced_state.Speeds.vy.value()}, swerve_synced_state.Speeds.omega.value() };
    }

    units::radians_per_second_t get_heading_rate() {
        return swerve_synced_state.Speeds.omega;
    }

    field_relative_output get_field_speeds() {
        return get_chassis_speeds().to_field_relative( get_heading() );
    }

    chassis_relative_output get_applied_output() {
        return swerve_hardware().get_applied_output();
    }

    chassis_relative_output get_applied_torque() {
        return chassis_relative_output{};
    }

    chassis_relative_output get_estimated_acceleration() {
        return chassis_relative_output{};
    }



    bool is_facing( units::radian_t angle, units::radian_t tolerance=3.683_deg ) {
        return units::math::abs( wrap_angle( angle - get_heading() ) ) < tolerance;
    }

    bool is_at_snap( units::radian_t tolerance=3.683_deg ) {
        return is_facing( heading_snap_target, tolerance );
    }

    bool is_at( vec2 point, units::meter_t tolerance=3.683_in ) {
        return ( get_vec2() - point ).length_squared() < ( tolerance.value() * tolerance.value() );
    }

    bool is_at_point( units::meter_t tolerance=3.683_in ) {
        return is_at( pid_to_point_target, tolerance);
    }

    bool collided( units::second_t within=30_ms ) {
        return time_since_last_collision.Get() < within;
    }

    void check_collisions() {
        static vec2 lastacceleration = vec2{0, 0};

        vec2 curacceleration = vec2{lowlevel_inst->GetPigeon2().GetAccelerationX().GetValueAsDouble(), lowlevel_inst->GetPigeon2().GetAccelerationY().GetValueAsDouble()};
        double curjerk = ((curacceleration - lastacceleration) / 0.01).length();

        vec2 scaled_speeds = get_chassis_speeds().xy_output / lowlevel_inst->strafe_freespeed.value();
        vec2 diff = scaled_speeds - get_applied_output().xy_output;

        if ( curjerk > k_swerve_jerk_threshold ) {
            time_since_last_collision.Restart();
        }
        lastacceleration = curacceleration;
    }

    void read_inputs() {
        swerve_synced_state = swerve_hardware().GetState();

        check_collisions();
    }

    void handle_states() {
        auto &hardware = swerve_hardware();

        switch( active_strafe_mode ) {

            case strafe_mode::X_LOCK: {
            } [[fallthrough]]; // fall through to IDLE for when turn_mode isn't in X_LOCK anymore

            case strafe_mode::IDLE: {
                active_strafe_constraints = strafe_constraints{};
                xy_out = vec2::zero;
                strafe_desaturation_bias = 0.0;
            } break;

            case strafe_mode::OPENLOOP: {
            } break;

            case strafe_mode::JOYSTICK: {
                double scale = teleop_strafe_speed;
                vec2 stick = oi::get_left_stick_driver( DEFAULT_DEADBAND, oi::input_shaping_function::CUSTOM );
                xy_out = stick * scale;
                active_strafe_constraints.max_accel = 17.5_mps_sq * (double)teleop_torque;
                active_strafe_constraints.max_speed = 4.5_mps;
            } break;

            case strafe_mode::PID_TO_POINT: {
                vec2 delta = pid_to_point_target - get_vec2();
                double err = delta.length();
                double feedback = 0;

                feedback = to_point_pid.calculate( err, active_pid_point_params.slot );

                xy_out = delta.hat() * ( feedback + active_pid_point_params.feedforward );
            } break;

            case strafe_mode::PID_LINE: {
                // TODO lookahead based on max accel?
                constexpr double lookahead_m = 0.6;

                vec2 pose = get_vec2();

                vec2 pose_to_target = ( pid_to_point_target - pose );
                vec2 parallel_diff = pose_to_target.projected_onto( pid_line_direction );
                vec2 cross_track = pose_to_target - parallel_diff;
                vec2 closest_line_point = pose + cross_track;

                double cross_track_error = cross_track.length();
                double parallel_error = parallel_diff.length();

                double line_lookahead = std::max( lookahead_m - cross_track_error, 0.0 );

                vec2 lookahead = closest_line_point.moved_towards( pid_to_point_target, line_lookahead );
                vec2 delta = lookahead - pose;

                double err = cross_track_error + parallel_error;

                double feedback = to_point_pid.calculate( err, active_pid_point_params.slot );

                xy_out = delta.hat() * ( feedback + active_pid_point_params.feedforward );
            } break;

            case strafe_mode::PID_ARC: {
                // TODO lookahead based on max accel?
                constexpr double lookahead_m = 0.6;

                vec2 pose = get_vec2();

                double arc_radius = ( pid_arc_center - pid_to_point_target ).length();
                vec2 center_to_target = pid_to_point_target - pid_arc_center;
                vec2 center_to_pose = pose - pid_arc_center;
                vec2 center_to_arc_point = center_to_pose.hat() * arc_radius;
                vec2 closest_arc_point = center_to_arc_point + pid_arc_center;
                double cross_track_error = ( closest_arc_point - pose ).length();
                double radial_error = wrap_angle( center_to_target.theta() - center_to_pose.theta() ).value() * arc_radius;

                double arc_lookahead = std::max( lookahead_m - cross_track_error, 0.0 );
                vec2 lookahead = pid_arc_center + center_to_arc_point.rotated_towards( center_to_target, units::radian_t{arc_lookahead / arc_radius} );
                vec2 delta = lookahead - pose;

                double err = cross_track_error + std::abs( radial_error );
                double feedback  = to_point_pid.calculate( err, active_pid_point_params.slot );

                xy_out = delta.hat() * ( feedback + active_pid_point_params.feedforward );
            } break;

            case strafe_mode::PID_SPEEDS: {
                // TODO
                active_strafe_constraints = strafe_constraints{};
                xy_out = vec2::zero;
                strafe_desaturation_bias = 0.0;
            } break;

            case strafe_mode::STTBOLLS: {
                // TODO
                active_strafe_constraints = strafe_constraints{};
                xy_out = vec2::zero;
                strafe_desaturation_bias = 0.0;
            } break;
        }


        switch( active_turn_mode ) {

            case turn_mode::X_LOCK: {
            } [[fallthrough]]; // fall through to IDLE for when strafe_mode isn't in X_LOCK anymore

            case turn_mode::IDLE: {

                active_turn_constraints = turn_constraints{};
                omega_out = 0.0;
                turn_desaturation_bias = 0.0;
                center_of_turning = vec2::zero;

            } break;

            case turn_mode::OPENLOOP: {
            } break;

            case turn_mode::JOYSTICK: {

                const double max_out = teleop_turn_speed;
                double right_x = oi::get_axis_driver( frc::XboxController::Axis::kRightX, DEFAULT_DEADBAND, oi::input_shaping_function::CUSTOM );
                omega_out = -right_x * max_out;
                active_turn_constraints.max_accel = 900_deg_per_s_sq * (double)teleop_torque;
                active_turn_constraints.max_speed = 600_deg_per_s;
                turn_desaturation_bias = 1.0;
                center_of_turning = vec2::zero;

            } break;

            case turn_mode::FACE_TARGET: {

                vec2 pose = get_vec2();
                vec2 towards_target = snap_target - pose;
                vec2 speeds = get_field_speeds().xy_output;
                heading_snap_target = towards_target.theta();

                vec2 speeds_towards_target = speeds.projected_onto( towards_target );
                vec2 speeds_perpendicular = speeds - speeds_towards_target;
                double direction = signum( towards_target.cross( speeds_perpendicular ) );
                units::radians_per_second_t target_omega = 1_rad_per_s * speeds_perpendicular.length() / towards_target.length() * (-direction);

                active_snap_params.feedforward_velocity = target_omega;
                active_snap_params.desaturation_bias = 0.1;

            } [[fallthrough]]; // fall through to SNAP

            case turn_mode::SNAP: {

                omega_out = heading_snap_pid.calculate(
                    get_heading().value(), pid_target{ heading_snap_target.value(), active_snap_params.feedforward_velocity.value() },
                    active_snap_params.slot
                );

                if( is_at_snap( active_snap_params.deadzone ) && active_snap_params.feedforward_velocity == 0_deg_per_s ) {
                    omega_out = 0;
                }

                turn_desaturation_bias = active_snap_params.desaturation_bias;
                center_of_turning = vec2::zero;

            } break;

            case turn_mode::PID_SPEED: {
                // TODO
                active_turn_constraints = turn_constraints{};
                omega_out = 0.0;
                turn_desaturation_bias = 0.0;
                center_of_turning = vec2::zero;
            } break;

            case turn_mode::STTBOLLS: {
                // TODO
                active_turn_constraints = turn_constraints{};
                omega_out = 0.0;
                turn_desaturation_bias = 0.0;
                center_of_turning = vec2::zero;
            } break;

            case turn_mode::FACE_STRAFE_DIR: {
                // TODO
                active_turn_constraints = turn_constraints{};
                omega_out = 0.0;
                turn_desaturation_bias = 0.0;
                center_of_turning = vec2::zero;
            } break;
        }

        // apply max strafe speed constraint
        vec2 field_speeds = get_field_speeds().xy_output;
        vec2 speeds = field_speeds.projected_onto( xy_out );
        double strafe_speed_percent = speeds.length() / hardware.strafe_freespeed.value();
        if( field_speeds.dot( xy_out ) < 0 ) { // current speeds point away from desired output
            strafe_speed_percent *= -1;
        }
        double max_xy_output = clamp_output( xy_out.length(), strafe_speed_percent, active_strafe_constraints.max_speed / hardware.strafe_freespeed, 1.0 );
        xy_out = xy_out.clamped( max_xy_output );

        // apply max turn speed constraint
        double omega_speed_percent = (double)( get_heading_rate() / hardware.omega_freespeed );
        double max_speed_percent = active_turn_constraints.max_speed / hardware.omega_freespeed;
        omega_out = clamp_output( omega_out, omega_speed_percent, max_speed_percent, 1.0 );
    }

    void set_outputs() {

        swerve_lowlevel &hardware = swerve_hardware();

        if( !swerve_enabled ) {

            hardware.set_control_disable();

        } else if( active_strafe_mode == strafe_mode::X_LOCK && active_turn_mode == turn_mode::X_LOCK ) {

            hardware.set_control_x_lock();

        } else {

            hardware.set_control( swerve_request{
                .target_output = field_relative_output{ xy_out, omega_out },
                .max_linear_accel = active_strafe_constraints.max_accel,
                .max_angular_accel = active_turn_constraints.max_accel,
                .turn_bias = 0.5 + turn_desaturation_bias - strafe_desaturation_bias,
                .center_of_rotation = center_of_turning,
            } );

        }
    }

    void update_dashboard() {
        swerve_hardware().update_nt();

        bool chillin_like_a_villan = true;
        for( auto &swerve_module : lowlevel_inst->GetModules() ) {
            int id = swerve_module->GetDriveMotor().GetDeviceID();
            double celcius = swerve_module->GetDriveMotor().GetDeviceTemp().GetValueAsDouble();
            frc::SmartDashboard::PutNumber( "health/drive_temp_" + std::to_string(id), celcius );
            if( celcius > 67 ) {
                chillin_like_a_villan = false;
            }
        }
        frc::SmartDashboard::PutBoolean( "health/drive_temperature_warning", chillin_like_a_villan );
    }



    void reset_heading( units::radian_t new_heading ) {
        swerve_hardware().ResetRotation( new_heading );
        heading_snap_pid.reset();
    }

    void reset_translation( vec2 new_translation ) {
        swerve_hardware().ResetTranslation( new_translation.t() );
        to_point_pid.reset();
    }

    void reset_odometry( pose2d new_pose ) {
        swerve_hardware().ResetPose( new_pose.in<units::meter_t>() );
        heading_snap_pid.reset();
        to_point_pid.reset();
    }




    void disable() {
        swerve_enabled = false;
    }

    void enable() {
        swerve_enabled = true;
    }

    void x_lock() {
        active_strafe_mode = strafe_mode::X_LOCK;
        active_turn_mode = turn_mode::X_LOCK;
    }



    void strafe_openloop( vec2 output, const strafe_constraints &constraints=strafe_constraints{}, double desaturation_bias=0.0 ) {
        active_strafe_mode = strafe_mode::OPENLOOP;
        xy_out = output;
        active_strafe_constraints = constraints;
        strafe_desaturation_bias = desaturation_bias;
    }

    void turn_openloop( double output, const turn_constraints &constraints=turn_constraints{}, double desaturation_bias=0.0, vec2 center_of_turning=vec2::zero ) {
        active_turn_mode = turn_mode::OPENLOOP;
        omega_out = output;
        active_turn_constraints = constraints;
        turn_desaturation_bias = desaturation_bias;
        swerve::center_of_turning = center_of_turning;
    }

    void drive_openloop( field_relative_output output, double turn_bias=0.0, vec2 center_of_turning=vec2::zero, 
        const strafe_constraints &sconstraints=strafe_constraints{}, const turn_constraints &tconstraints=turn_constraints{} )
    {
        strafe_openloop( output.xy_output, sconstraints, 0.0 );
        turn_openloop( output.omega_output, tconstraints, turn_bias, center_of_turning );
    }



    void strafe_idle() {
        active_strafe_mode = strafe_mode::IDLE;
    }

    void turn_idle() {
        active_turn_mode = turn_mode::IDLE;
    }



    void strafe_tele() {
        active_strafe_mode = strafe_mode::JOYSTICK;
    }

    void turn_tele() {
        active_turn_mode = turn_mode::JOYSTICK;
    }



    void pid_to_point( vec2 target, const pid_to_point_params &params=pid_to_point_params{}, const strafe_constraints &constraints=strafe_constraints{} ) {
        if( active_strafe_mode != strafe_mode::PID_TO_POINT || ( target - pid_to_point_target ).length_squared() > 1 ) {
            to_point_pid.reset();
        }
        active_strafe_mode = strafe_mode::PID_TO_POINT;
        pid_to_point_target = target;
        active_pid_point_params = params;
        active_strafe_constraints = constraints;
    }

    void pid_arc( vec2 target, vec2 arc_center, const pid_to_point_params &params=pid_to_point_params{}, const strafe_constraints &constraints=strafe_constraints{} ) {
        if( active_strafe_mode != strafe_mode::PID_ARC || ( target - pid_to_point_target ).length_squared() > 1 ) {
            to_point_pid.reset();
        }
        active_strafe_mode = strafe_mode::PID_ARC;
        pid_to_point_target = target;
        pid_arc_center = arc_center;
        active_pid_point_params = params;
        active_strafe_constraints = constraints;
    }

    void pid_line( vec2 target, vec2 direction, const pid_to_point_params &params=pid_to_point_params{}, const strafe_constraints &constraints=strafe_constraints{} ) {
        if( active_strafe_mode != strafe_mode::PID_LINE || ( target - pid_to_point_target ).length_squared() > 1 ) {
            to_point_pid.reset();
        }
        active_strafe_mode = strafe_mode::PID_LINE;
        pid_to_point_target = target;
        pid_line_direction = direction;
        active_pid_point_params = params;
        active_strafe_constraints = constraints;
    }

    void snap_heading( units::radian_t target, const heading_snap_params &params=heading_snap_params{}, const turn_constraints &constraints=turn_constraints{} ) {
        if( units::math::abs( wrap_angle( heading_snap_target - target ) ) > 45_deg || active_turn_mode != turn_mode::SNAP ) {
            heading_snap_pid.reset();
        }
        active_turn_mode = turn_mode::SNAP;
        heading_snap_target = target;
        active_snap_params = params;
        active_turn_constraints = constraints;
    }

    void snap_towards( vec2 target, units::radian_t offset=0_deg, const heading_snap_params &params=heading_snap_params{}, const turn_constraints &constraints=turn_constraints{} ) {
        if( (target - snap_target).length_squared() > 0.5 || active_turn_mode != turn_mode::FACE_TARGET ) {
            heading_snap_pid.reset();
        }
        active_turn_mode = turn_mode::FACE_TARGET;
        snap_target = target;
        snap_target_offset = offset;
        active_snap_params = params;
        active_turn_constraints = constraints;
    }

}
