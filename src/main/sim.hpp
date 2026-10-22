#include <span>
#include <functional>

#include <networktables/StructArrayTopic.h>
#include <frc/geometry/Pose3d.h>

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/DCMotor.h>

#include "ballz_sim.hpp"

using get_sim_func = std::function<ctre::phoenix6::sim::TalonFXSimState&()>;

get_sim_func get_owl_sim = nullptr;
get_sim_func get_hood_sim = nullptr;
get_sim_func get_flywheel_sim = nullptr;
get_sim_func get_pivot_sim = nullptr;
get_sim_func get_neck_sim = nullptr;

static nt::StructArrayPublisher<frc::Pose3d> finals = nt::NetworkTableInstance::GetDefault()
    .GetStructArrayTopic<frc::Pose3d>( "FinalComponentPoses" ).Publish();

static nt::StructArrayPublisher<frc::Pose3d> ballz = nt::NetworkTableInstance::GetDefault()
    .GetStructArrayTopic<frc::Pose3d>( "ballz" ).Publish();

static double owl_gear = cfg::owl_config.Feedback.SensorToMechanismRatio;
static double hood_gear = cfg::hood_cfg.Feedback.SensorToMechanismRatio;
static double flywheel_gear = cfg::shooter_config.Feedback.SensorToMechanismRatio;
static frc::sim::DCMotorSim owl_sim_model{
    frc::LinearSystemId::DCMotorSystem(
        frc::DCMotor::KrakenX44FOC(1),
        0.0403_kg_sq_m,
        owl_gear
    ),
    frc::DCMotor::KrakenX44FOC(1)
};
static frc::sim::DCMotorSim hood_sim_model{
    frc::LinearSystemId::DCMotorSystem(
        frc::DCMotor::KrakenX44FOC(1),
        0.005_kg_sq_m,
        hood_gear
    ),
    frc::DCMotor::KrakenX44FOC(1)
};
static frc::sim::DCMotorSim flywheel_sim_model{
    frc::LinearSystemId::DCMotorSystem(
        frc::DCMotor::KrakenX60FOC(2),
        k_flywheel_moi,
        flywheel_gear
    ),
    frc::DCMotor::KrakenX60FOC(2)
};

static double intake_pivot_gear = cfg::intake::pivot_config.Feedback.SensorToMechanismRatio;
static frc::sim::DCMotorSim intake_pivot_sim_model{
    frc::LinearSystemId::DCMotorSystem(
        frc::DCMotor::KrakenX44FOC(1),
        0.067_kg_sq_m,
        intake_pivot_gear
    ),
    frc::DCMotor::KrakenX44FOC(1)
};

static double neck_gear = 48.0/10.0;
frc::sim::DCMotorSim neck_sim_model{
    frc::LinearSystemId::DCMotorSystem(
        frc::DCMotor::KrakenX60FOC(1),
        0.01_kg_sq_m,
        neck_gear
    ),
    frc::DCMotor::KrakenX60FOC(1)
};

static void handle_sim_talon( get_sim_func getter, frc::sim::DCMotorSim &sim_model, double gearing ) {
    if( !getter ) {
        return;
    }
    auto &talonFXSim = getter();

    talonFXSim.SetSupplyVoltage( frc::RobotController::GetBatteryVoltage() );

    auto motorVoltage = talonFXSim.GetMotorVoltage();

    sim_model.SetInputVoltage(motorVoltage);
    sim_model.Update( 10_ms );

    talonFXSim.SetRawRotorPosition( gearing * sim_model.GetAngularPosition());
    talonFXSim.SetRotorVelocity( gearing * sim_model.GetAngularVelocity());
}

void sim_init() {
    if( get_owl_sim ) {
        get_owl_sim().SetMotorType( ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX44 );
    }
    if( get_hood_sim ) {
        get_hood_sim().SetMotorType( ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX44 );
    }
    if( get_flywheel_sim ) {
        get_flywheel_sim().SetMotorType( ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60 );
    }
    if( get_pivot_sim ) {
        get_pivot_sim().SetMotorType( ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX44 );
    }
    if( get_neck_sim ) {
        get_neck_sim().SetMotorType( ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX60 );
        get_neck_sim().Orientation = ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    }
}

void sim_periodic() {

    static units::inch_t intake_slide = 12_in;

    handle_sim_talon( get_owl_sim, owl_sim_model, owl_gear );
    handle_sim_talon( get_hood_sim, hood_sim_model, hood_gear );
    handle_sim_talon( get_flywheel_sim, flywheel_sim_model, flywheel_gear );
    handle_sim_talon( get_pivot_sim, intake_pivot_sim_model, intake_pivot_gear );
    handle_sim_talon( get_neck_sim, neck_sim_model, neck_gear );

    frc::SmartDashboard::PutNumber( "flywheel_sim_rpm", units::revolutions_per_minute_t{flywheel_sim_model.GetAngularVelocity()}.value() );
    frc::SmartDashboard::PutNumber( "neck_sim_rpm", units::revolutions_per_minute_t{neck_sim_model.GetAngularVelocity()}.value() );

    simulate_balls( flywheel_sim_model, neck_sim_model, hood_sim_model, owl_sim_model );

    auto turret_deg = owl_sim_model.GetAngularPosition();

    vec2 turret_offset{ -4.25, 0 };
    vec2 hood_pivot_offset = turret_offset + vec2{ 4, units::radian_t{turret_deg} };

    frc::Pose3d poses[] = {
        frc::Pose3d{ frc::Translation3d{ 13.5_in, 0_in, 6.8_in+2.75_in }, frc::Rotation3d{ 0_deg, -intake_pivot_sim_model.GetAngularPosition(), 0_deg } },
        frc::Pose3d{ frc::Translation3d{ intake_slide, 0_in, 0_in }, frc::Rotation3d{} },
        frc::Pose3d{ frc::Translation3d{ units::inch_t{turret_offset.x}, 0_in, 0_in }, frc::Rotation3d{ 0_deg, 0_deg, turret_deg } },
        frc::Pose3d{ frc::Translation3d{ units::inch_t{hood_pivot_offset.x}, units::inch_t{hood_pivot_offset.y}, 18.25_in+2.75_in }, frc::Rotation3d{ 0_deg, -hood_sim_model.GetAngularPosition(), turret_deg } },
    };
    finals.Set( poses );

    ballz.Set( simulated_balls_positions );
}

