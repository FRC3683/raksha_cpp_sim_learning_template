#include "dave/swerve/swerve_config.hpp"


template<>
ctre::phoenix6::swerve::SwerveDrivetrainConstants swerve_cfg<4>::drivetrain_constants() const {
    auto pigeon_cfg = ctre::phoenix6::configs::Pigeon2Configuration{}
        .WithMountPose( hardware_cfg.pigeon_mounting )
        .WithGyroTrim( tuning_cfg.gyro_trim )
    ;

    return ctre::phoenix6::swerve::SwerveDrivetrainConstants{}
        .WithCANBusName( software_cfg.canbus.GetName() )
        .WithPigeon2Id( software_cfg.pigeon_id )
        .WithPigeon2Configs( pigeon_cfg )
    ;
}

template<>
ctre_module_constants swerve_cfg<4>::module_constants( size_t i ) const {
    using namespace ctre::phoenix6::configs;
    using namespace ctre::phoenix6::swerve;

    ctre_module_constants constants;

    constants.CouplingGearRatio = hardware_cfg.type.couple_ratio;

    constants.DriveMotorGearRatio = hardware_cfg.type.drive_ratio;
    constants.DriveMotorId = software_cfg.module_cfgs[i].drive_id;
    constants.DriveMotorInitialConfigs
        .WithOpenLoopRamps( OpenLoopRampsConfigs{}
            .WithVoltageOpenLoopRampPeriod( 0.01_s )
        )
        .WithAudio( AudioConfigs{}
            .WithAllowMusicDurDisable( true )
        )
    ;
    constants.DriveMotorInverted = software_cfg.module_cfgs[i].drive_invert;
    constants.DriveMotorType = DriveMotorArrangement::TalonFX_Integrated;

    constants.EncoderId = software_cfg.module_cfgs[i].cancoder_id;
    constants.EncoderInverted = software_cfg.module_cfgs[i].cancoder_invert;
    constants.EncoderOffset = software_cfg.module_cfgs[i].cancoder_magnet_offset;
    constants.FeedbackSource = software_cfg.is_pro_licensed ? SteerFeedbackType::FusedCANcoder : SteerFeedbackType::RemoteCANcoder;

    constants.LocationX = units::meter_t{ hardware_cfg.module_positions[i].x };
    constants.LocationY = units::meter_t{ hardware_cfg.module_positions[i].y };

    constants.SlipCurrent = tuning_cfg.drive_current_limits.StatorCurrentLimit;
    constants.SpeedAt12Volts = hardware_cfg.free_speed();
    constants.SteerMotorClosedLoopOutput = ClosedLoopOutputType::Voltage;
    constants.SteerMotorGains
        .WithKP( tuning_cfg.steerKP )
        .WithKD( tuning_cfg.steerKD )
        .WithKV( tuning_cfg.steerKV )
    ;
    constants.SteerMotorGearRatio = hardware_cfg.type.steer_ratio;
    constants.SteerMotorId = software_cfg.module_cfgs[i].steer_id;
    constants.SteerMotorInitialConfigs
        .WithAudio( AudioConfigs{}
            .WithAllowMusicDurDisable( true )
        )
    ;
    constants.SteerMotorInverted = software_cfg.module_cfgs[i].steer_invert;
    constants.SteerMotorType = SteerMotorArrangement::TalonFX_Integrated;

    constants.WheelRadius = hardware_cfg.type.wheel_radius * tuning_cfg.wheel_k;

    constants.DriveInertia = 0.04_kg_sq_m;
    constants.SteerInertia = 0.00267_kg_sq_m;

    return constants;
}


/**
 * module order: front left, back left, back right, front right
 */
swerve_hardware_cfg<4> rectangle_frame_x60( const swerve_module_type &module_type, const units::meter_t wheelbase, const units::meter_t trackwidth, 
    const ctre::phoenix6::configs::MountPoseConfigs pigeon_mounting,
    units::meters_per_second_squared_t peak_linear_accel,
    units::radians_per_second_squared_t peak_angular_accel )
{
    const double half_wb = wheelbase.value() / 2.0f;
    const double half_tw = trackwidth.value() / 2.0f;
    return swerve_hardware_cfg<4>{
        .type = module_type,
        .module_positions = {
            vec2{ half_wb, half_tw }, // front left
            vec2{ -half_wb, half_tw }, // back left
            vec2{ -half_wb, -half_tw }, // back right
            vec2{ half_wb, -half_tw }, // front right
        },
        .pigeon_mounting = pigeon_mounting,
        .drive_motor_freespeed = 5800_rpm,
        .drive_motor_stall_torque = 476_A,
        .peak_linear_accel = peak_linear_accel,
        .peak_angular_accel = peak_angular_accel
    };
}

/**
 * module order: front left, back left, back right, front right
 */
swerve_software_cfg<4> rectangle_frame( ctre::phoenix6::CANBus &canbus,
            const swerve_module_cfg &front_left_cfg, const swerve_module_cfg &back_left_cfg,
            const swerve_module_cfg &back_right_cfg, const swerve_module_cfg &front_right_cfg,
            const int pigeon_id, const bool is_pro_licensed ) {
    return swerve_software_cfg<4>{
        .canbus = canbus,
        .module_cfgs = { front_left_cfg, back_left_cfg, back_right_cfg, front_right_cfg },
        .pigeon_id = pigeon_id,
        .is_pro_licensed = is_pro_licensed
    };
}

const swerve_module_type MK4_L1{ 
    .drive_ratio = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0),
    .steer_ratio = 150.0 / 7.0 ,
    .couple_ratio = (25.0 / 19.0) * (15.0 / 45.0),
    .wheel_radius = 2_in
};
const swerve_module_type MK4_L2{ 
    .drive_ratio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0),
    .steer_ratio = 150.0 / 7.0 ,
    .couple_ratio = (27.0 / 17.0) * (15.0 / 45.0),
    .wheel_radius = 2_in
};
const swerve_module_type MK4_L3{ 
    .drive_ratio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0),
    .steer_ratio = 150.0 / 7.0 ,
    .couple_ratio = (28.0 / 16.0) * (15.0 / 45.0),
    .wheel_radius = 2_in
};
const swerve_module_type MK5_R1{ 
    .drive_ratio = (54.0 / 12.0) * (25.0 / 32.0) * (30.0 / 15.0),
    .steer_ratio = 287.0 / 11.0,
    .couple_ratio = (32.0 / 25.0) * (15.0 / 30.0),
    .wheel_radius = 2_in
};
const swerve_module_type MK5_R2{ 
    .drive_ratio = (54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0),
    .steer_ratio = 287.0 / 11.0,
    .couple_ratio = (32.0 / 25.0) * (15.0 / 30.0),
    .wheel_radius = 2_in
};
const swerve_module_type MK5_R3{ 
    .drive_ratio = (54.0 / 16.0) * (25.0 / 32.0) * (30.0 / 15.0),
    .steer_ratio = 287.0 / 11.0,
    .couple_ratio = (32.0 / 25.0) * (15.0 / 30.0),
    .wheel_radius = 2_in
};
