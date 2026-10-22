#pragma once

#include <dave/ctre_common.hpp>
#include <dave/units_common.hpp>
#include <dave/math.hpp>
#include <dave/swerve/swerve_config.hpp>
#include <dave/pid.hpp>

#include "constants.hpp"


namespace cfg {

    ctre::phoenix6::CANBus canivore{ "canivore" };
    ctre::phoenix6::CANBus rio{ "rio" };

    //shooter can ID's
    constexpr int can_id_turn_shooter = 20;
    constexpr int can_id_hood = 21;
    constexpr int can_id_flylead = 22;
    constexpr int can_id_flyslave = 23;

    constexpr int can_id_sixteen_encoder = 24;
    constexpr int can_id_seventeen_encoder = 25;


    constexpr int can_front_left_drive  = 2;
    constexpr int can_front_left_steer  = 1;
    constexpr int can_front_left_cancoder  = 11;

    constexpr int can_back_left_drive  = 4;
    constexpr int can_back_left_steer  = 3;
    constexpr int can_back_left_cancoder  = 12;

    constexpr int can_back_right_drive  = 6;
    constexpr int can_back_right_steer  = 5;
    constexpr int can_back_right_cancoder  = 13;

    constexpr int can_front_right_drive  = 8;
    constexpr int can_front_right_steer  = 7;
    constexpr int can_front_right_cancoder  = 14;

    constexpr int can_pigeon = 10;

    constexpr int can_candle = 50;

    constexpr int can_intake_pivot = 17;
    constexpr int can_intake_roller_master = 18;
    constexpr int can_intake_roller_slave = 19;

    using namespace ctre::phoenix6::configs;

    bool is_comp = true;


    TalonFXConfiguration hood_cfg =  TalonFXConfiguration{}
            .WithFeedback( FeedbackConfigs {}
                .WithSensorToMechanismRatio( ( 294.0 / 20.0 ) * ( 42.0 / 18.0 ) * ( 68.0 / 9.0 ) )
            )
            .WithMotorOutput( MotorOutputConfigs{}
                .WithNeutralMode( ctre::phoenix6::signals::NeutralModeValue::Brake)
            )
            .WithSlot0( Slot0Configs{}
                .WithKP( 670 )
            )
            .WithCurrentLimits( CurrentLimitsConfigs {}
                .WithStatorCurrentLimit( 50_A )
                .WithSupplyCurrentLimit( 50_A )
            )
            .WithSoftwareLimitSwitch( SoftwareLimitSwitchConfigs {}
                .WithForwardSoftLimitEnable(true)
                .WithReverseSoftLimitEnable(true)
                .WithForwardSoftLimitThreshold( hood_min_angle * 1_deg)
                .WithReverseSoftLimitThreshold( hood_max_angle * 1_deg)
            )
        ;


    TalonFXConfiguration owl_config = TalonFXConfiguration{}
        .WithFeedback( FeedbackConfigs{}
            .WithSensorToMechanismRatio( (160.0 / 16.0) * (54.0 / 18.0) * (24.0 / 12.0) )
        )
        .WithSlot0( Slot0Configs{}
            .WithKP( 167 )
            .WithKA( 0.1067 )
            .WithKV( 5.867 + 1.67 + 2.67 )
            .WithKS( 0.39 )
        )
        .WithCurrentLimits( CurrentLimitsConfigs{}
            .WithStatorCurrentLimit( 85_A )
            .WithSupplyCurrentLimit( 40_A )
        )
        .WithMotorOutput( MotorOutputConfigs{}
            .WithInverted( ctre::phoenix6::signals::InvertedValue::Clockwise_Positive )
        )
        .WithSoftwareLimitSwitch( SoftwareLimitSwitchConfigs{}
            .WithForwardSoftLimitEnable(true)
            .WithReverseSoftLimitEnable(true)
            .WithForwardSoftLimitThreshold( 558_deg )
            .WithReverseSoftLimitThreshold( 9_deg )
        )
    ;

    TalonFXConfiguration shooter_slave_config = TalonFXConfiguration{}
        .WithFeedback( FeedbackConfigs{}
            .WithSensorToMechanismRatio(1)
        )
        .WithSlot0( Slot0Configs{}
            .WithKP( 10 )
            .WithKG( 0.05 )
        )
        .WithTorqueCurrent(TorqueCurrentConfigs{}
            .WithPeakForwardTorqueCurrent( 120_A )
            .WithPeakReverseTorqueCurrent( -120_A )
        )
        .WithCurrentLimits(CurrentLimitsConfigs{}
            .WithSupplyCurrentLimit( 40_A )
        );
        
    TalonFXConfiguration shooter_config = TalonFXConfiguration{}
        .WithFeedback( FeedbackConfigs{}
            .WithSensorToMechanismRatio(1)
        )
        .WithSlot0( Slot0Configs{}
            .WithKP( 10 )
            .WithKG( 0.05 )
        )
        .WithTorqueCurrent(TorqueCurrentConfigs{}
            .WithPeakForwardTorqueCurrent( 120_A )
            .WithPeakReverseTorqueCurrent( -120_A )
        )
        .WithCurrentLimits(CurrentLimitsConfigs{}
            .WithSupplyCurrentLimit( 40_A )
        );

    namespace intake {

        TalonFXConfiguration pivot_config = TalonFXConfiguration{}
            .WithSlot0( Slot0Configs{}
                .WithKP( 200 )
            )
            .WithSlot1( Slot1Configs{}
                .WithKP( 10 )
                .WithGainSchedBehavior( GainSchedBehaviorValue::ZeroOutput )
            )
            .WithClosedLoopGeneral( ClosedLoopGeneralConfigs{}
                .WithGainSchedErrorThreshold( 5_deg )
            )
            .WithMotionMagic( MotionMagicConfigs{}
                .WithMotionMagicAcceleration( 8_tr_per_s_sq )
                .WithMotionMagicCruiseVelocity( 1.5_tps )
            )
            .WithCurrentLimits( CurrentLimitsConfigs{}
                .WithStatorCurrentLimit( 120_A )
            )
            .WithFeedback( FeedbackConfigs{}
                .WithSensorToMechanismRatio( (30.0 / 12.0) * (64.0 / 16.0) * (50.0 / 20.0) * (36.0 / 12.0) )
            )
            .WithSoftwareLimitSwitch( SoftwareLimitSwitchConfigs{}
                .WithForwardSoftLimitEnable( true )
                .WithReverseSoftLimitEnable( true )
                .WithForwardSoftLimitThreshold( k_intake_hardstop )
                .WithReverseSoftLimitThreshold( k_intake_deployed )
            )
            .WithMotorOutput( MotorOutputConfigs{}
                .WithInverted( InvertedValue::Clockwise_Positive )
            )
        ;

        TalonFXConfiguration roller_config = TalonFXConfiguration{}
            .WithMotorOutput( MotorOutputConfigs{}
                .WithInverted( InvertedValue::CounterClockwise_Positive )
            )
            .WithOpenLoopRamps( OpenLoopRampsConfigs{}
                .WithVoltageOpenLoopRampPeriod( 200_ms )
            )
            .WithCurrentLimits( CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit( 30_A )
                .WithStatorCurrentLimit( 65_A )
            )
        ;

    }

    namespace swerve {
        pid_config heading_snap_cfg = {
            .slot0{ .kGain = 0.8, .kI = 0.0, .kD = 0.067, .kV = 0.12 },
            .continuous = true, .min_input = -PI, .max_input = PI
        };

        pid_config pid_to_point_cfg = {
            .slot0{ .kGain = 2.5, .kI = 0.0, .kD = 0.02 },
            .slot1{ .kGain = 1.4, .kI = 0.0, .kD = 0.105 }
        };

        swerve_cfg<4> lowlevel_cfg = {
            .hardware_cfg = rectangle_frame_x60( MK5_R1, 25.25_in, 18.25_in, 
                ctre::phoenix6::configs::MountPoseConfigs{}
                    .WithMountPoseYaw( -90_deg ),
                17.5_mps_sq, // peak accel
                1000_deg_per_s_sq // peak accel
            ),
            .tuning_cfg = swerve_tuning_cfg{
                .drive_current_limits = ctre::phoenix6::configs::CurrentLimitsConfigs{}
                    .WithStatorCurrentLimit( 120_A ),
                .steer_current_limits = ctre::phoenix6::configs::CurrentLimitsConfigs{}
                    .WithStatorCurrentLimit( 120_A )
                    .WithSupplyCurrentLimit( 60_A )
                    .WithSupplyCurrentLowerLimit( 40_A ),
                .steerKP = 100.67,
                .steerKD = 0.0,
                .steerKV = 3.267,
                .wheel_k = 1.0,
                .antiskew = 0.067,
                .coupling_kF = 0.0_V,
                .max_coupling_output = 0.0_V,
                .gyro_trim = ctre::phoenix6::configs::GyroTrimConfigs{}
                    .WithGyroScalarZ( -2 ),
            },
            .software_cfg = rectangle_frame(
                canivore,
                // FRONT LEFT
                swerve_module_cfg{
                    .steer_id = can_front_left_steer,
                    .drive_id = can_front_left_drive,
                    .cancoder_id = can_front_left_cancoder,
                    .steer_invert = false,
                    .drive_invert = true,
                    .cancoder_invert = false,
                    .cancoder_magnet_offset = -0.169677734375_tr
                },
                // BACK LEFT
                swerve_module_cfg{
                    .steer_id = can_back_left_steer,
                    .drive_id = can_back_left_drive,
                    .cancoder_id = can_back_left_cancoder,
                    .steer_invert = false,
                    .drive_invert = true,
                    .cancoder_invert = false,
                    .cancoder_magnet_offset = -0.07080078125_tr
                },
                // BACK RIGHT
                swerve_module_cfg{
                    .steer_id = can_back_right_steer,
                    .drive_id = can_back_right_drive,
                    .cancoder_id = can_back_right_cancoder,
                    .steer_invert = false,
                    .drive_invert = true,
                    .cancoder_invert = false,
                    .cancoder_magnet_offset = 0.272216796875_tr
                },
                // FRONT RIGHT
                swerve_module_cfg{
                    .steer_id = can_front_right_steer,
                    .drive_id = can_front_right_drive,
                    .cancoder_id = can_front_right_cancoder,
                    .steer_invert = false,
                    .drive_invert = true,
                    .cancoder_invert = false,
                    .cancoder_magnet_offset = 0.3193359375_tr
                },
                can_pigeon,
                true // pro licensed
            )
        };
    }


    void prac_bot_configs() {
        is_comp = false;

        // practice bot has different swerve cancoder magnet offsets:
        swerve::lowlevel_cfg.software_cfg.module_cfgs[0].cancoder_magnet_offset = -0.169677734375_tr; // front left
        swerve::lowlevel_cfg.software_cfg.module_cfgs[1].cancoder_magnet_offset = -0.07080078125_tr; // back left
        swerve::lowlevel_cfg.software_cfg.module_cfgs[2].cancoder_magnet_offset = 0.272216796875_tr; // back right
        swerve::lowlevel_cfg.software_cfg.module_cfgs[3].cancoder_magnet_offset = 0.3193359375_tr; // front right

        // // example: reconfiguring pid for smth:
        // swerve::heading_snap_cfg.slot0.kGain = 1;
        // // example: practice bot has different swerves? pls no
        // swerve::lowlevel_cfg.hardware_cfg.type = MK4_L1;
    }
};
