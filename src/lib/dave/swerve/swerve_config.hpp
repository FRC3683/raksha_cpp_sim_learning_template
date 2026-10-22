#pragma once

#include <units/voltage.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/frequency.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/configs/CurrentLimitsConfigs.hpp>
#include <ctre/phoenix6/configs/MountPoseConfigs.hpp>
#include <ctre/phoenix6/swerve/SwerveDrivetrainConstants.hpp>
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>

#include "dave/vector.hpp"

typedef ctre::phoenix6::swerve::SwerveModuleConstants<ctre::phoenix6::configs::TalonFXConfiguration, ctre::phoenix6::configs::TalonFXConfiguration,ctre::phoenix6::configs::CANcoderConfiguration> ctre_module_constants;

struct swerve_module_type {
    double drive_ratio, steer_ratio, couple_ratio;
    units::meter_t wheel_radius;
};

extern const swerve_module_type MK4_L1, MK4_L2, MK4_L3, MK5_R1, MK5_R2, MK5_R3;

template<unsigned int num_modules>
struct swerve_hardware_cfg {
    swerve_module_type type;
    vec2 module_positions[num_modules];
    ctre::phoenix6::configs::MountPoseConfigs pigeon_mounting;
    units::revolutions_per_minute_t drive_motor_freespeed;
    units::ampere_t drive_motor_stall_torque;
    units::meters_per_second_squared_t peak_linear_accel;
    units::radians_per_second_squared_t peak_angular_accel;

    units::meters_per_second_t free_speed() const {
        return ( units::radians_per_second_t{ drive_motor_freespeed } / type.drive_ratio ).value() * type.wheel_radius / 1.0_s;
    }

    double calc_chassis_radius() const {
        double max_length_squared = 0;
        for( const vec2 &vec : module_positions ) {
            double l = vec.length_squared();
            if( l > max_length_squared ) {
                max_length_squared = l;
            }
        }
        return std::sqrt( max_length_squared );
    }
};

swerve_hardware_cfg<4> rectangle_frame_x60( const swerve_module_type &module_type, const units::meter_t wheelbase, const units::meter_t trackwidth, 
    const ctre::phoenix6::configs::MountPoseConfigs pigeon_mounting,
    units::meters_per_second_squared_t peak_linear_accel,
    units::radians_per_second_squared_t peak_angular_accel );

struct swerve_module_cfg {
    int steer_id, drive_id, cancoder_id;
    bool steer_invert, drive_invert, cancoder_invert;
    units::turn_t cancoder_magnet_offset;
};

struct swerve_tuning_cfg {
    ctre::phoenix6::configs::CurrentLimitsConfigs drive_current_limits, steer_current_limits;
    double steerKP, steerKD, steerKV;
    double wheel_k;
    double antiskew;
    units::volt_t coupling_kF, max_coupling_output;
    ctre::phoenix6::configs::GyroTrimConfigs gyro_trim = ctre::phoenix6::configs::GyroTrimConfigs{};
};

template<unsigned int num_modules>
struct swerve_software_cfg {
    ctre::phoenix6::CANBus &canbus;
    swerve_module_cfg module_cfgs[num_modules];
    int pigeon_id;
    bool is_pro_licensed;

    const units::hertz_t odom_freq() const {
        return units::hertz_t{ canbus.IsNetworkFD() ? 250.0 : 100.0 };
    }
};

swerve_software_cfg<4> rectangle_frame( ctre::phoenix6::CANBus &canbus,
            const swerve_module_cfg &front_left_cfg, const swerve_module_cfg &back_left_cfg,
            const swerve_module_cfg &back_right_cfg, const swerve_module_cfg &front_right_cfg,
            const int pigeon_id, const bool is_pro_licensed );

template<unsigned int num_modules>
struct swerve_cfg {
    swerve_hardware_cfg<num_modules> hardware_cfg;
    swerve_tuning_cfg tuning_cfg;
    swerve_software_cfg<num_modules> software_cfg;

    ctre::phoenix6::swerve::SwerveDrivetrainConstants drivetrain_constants() const;
    ctre_module_constants module_constants( size_t i ) const;
    
};
