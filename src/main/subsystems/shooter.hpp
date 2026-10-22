#pragma once

#include "dave/ctre_common.hpp"
#include "dave/units_common.hpp"
#include "swerve.hpp"
#include <thread>

#include "dave/filters/debouncer.hpp"
#include <frc/controller/PIDController.h>
#include "config.hpp"
#include <wpi/interpolating_map.h> 
#include "magic_enum/magic_enum.hpp"


using namespace frc;

namespace shooter {
    enum class state {
        //A = autonomous
        //M = mannually
        ASHOOT,
        ATRACK,
        MSHOOT,
        MTRACK,
        RAGDOLL,
        ZEROING
    };

    wpi::interpolating_map<double, double> dist_and_rpm;
    wpi::interpolating_map<double, double> dist_and_hood_angle;

    void populate_interpolating_map () {
        for (int i = 0; i < std::size(shotmap); i = i + 3) {
            dist_and_rpm.insert(shotmap[i]*0.0254, shotmap[i+2]);
            dist_and_hood_angle.insert(shotmap[i]*0.0254, shotmap[i+1]);
        }
    }

    double cur_target_angle = 180;
    CANcoder sixteen {cfg::can_id_sixteen_encoder, "canivore"};
    CANcoder seventeen = CANcoder{ cfg::can_id_seventeen_encoder, "canivore"};// {25, "can"};
    
    
    std::vector<double> possible_pose_based_on_sixteen;
    std::vector<double> possible_pose_based_on_seventeen;

    ctre::phoenix6::hardware::TalonFX turn {cfg::can_id_turn_shooter, "canivore"};
    ctre::phoenix6::hardware::TalonFX hood_motor {cfg::can_id_hood, "canivore"};
    ctre::phoenix6::hardware::TalonFX leader_shooter_motor {cfg::can_id_flylead, "canivore"};
    ctre::phoenix6::hardware::TalonFX enslaved_shooter_motor {cfg::can_id_flyslave, "canivore"};


    state curstate = state::RAGDOLL;

    ctre::phoenix6::configs::Slot0Configs turnpidConfigs;

    PositionVoltage turn_request {0.0_deg};

    debouncer hood_zeroer = debouncer::rising_edge(300_ms, false);

    bool zeroed = false;

    void apply_configs (double k_p, double k_i, double k_d, double k_v, ctre::phoenix6::hardware::TalonFX& motor, ctre::phoenix6::configs::Slot0Configs pidConfigs, double gear_ratio) {
        pidConfigs.kP = k_p;  // Proportional gain
        pidConfigs.kI = k_i;  // Integral gain
        pidConfigs.kD = k_d;  // Derivative gain
        pidConfigs.kV = k_v;
        // Apply configuration to the motor
        motor.GetConfigurator().Apply(pidConfigs);

        ctre::phoenix6::configs::TalonFXConfiguration motorConfigs{};

        motorConfigs.Feedback.SensorToMechanismRatio = gear_ratio;
        motor.GetConfigurator().Apply(motorConfigs);
    }

    void six_teen () {
        double sixteen_rot = sixteen.GetAbsolutePosition().GetValueAsDouble();
        double sixteen_teeth = sixteen_rot*16;

        for(int i = 0; i < (17); i++) {
            possible_pose_based_on_sixteen.push_back(16*(i)+sixteen_teeth);
        }
    }

    void seven_teen () {
        double seventeen_rot = seventeen.GetAbsolutePosition().GetValueAsDouble();
        double seventeen_teeth = seventeen_rot*17;

        for(int i = 0; i < (16); i++) {
            possible_pose_based_on_seventeen.push_back(17*(i)+seventeen_teeth);
        }
    }

    units::degree_t get_hood_angle() {
        return hood_motor.GetPosition().GetValue();
    }

    units::degree_t get_owl_angle() {
        return turn.GetPosition().GetValue() - (306.85_deg - 180_deg);
    }



    /**
    * \return returns the offset off zero in degrees
    */

    double calculate_crt_angle() {
        possible_pose_based_on_seventeen.clear();
        possible_pose_based_on_sixteen.clear();
        six_teen();
        seven_teen();

        double smallest_diff = 10000000;
        int smallest_diff_sixteen = 0; 
        int smallest_diff_seventeen = 0; 
        for(int i = 0; i < (possible_pose_based_on_sixteen.size()); i++) {
            for(int j = 0; j < (possible_pose_based_on_seventeen.size()); j++) {
                double abs_diff = std::abs( possible_pose_based_on_sixteen.at(i) - possible_pose_based_on_seventeen.at(j) );
                if( abs_diff < smallest_diff ) {
                    smallest_diff = abs_diff;
                    smallest_diff_seventeen = j;
                    smallest_diff_sixteen = i; 
                }
            }
        }
        return (((possible_pose_based_on_sixteen.at(smallest_diff_sixteen) + possible_pose_based_on_seventeen.at(smallest_diff_seventeen))/2)/160*360);
    }
    void zero_turn_pose () {
        if( frc::RobotBase::IsSimulation() ) {
            return;
        }
        turn.SetPosition(units::degree_t(calculate_crt_angle()));
    
    };

    void snapp_robot_relative(double target_deg) {
        while (target_deg >= 360) {
            target_deg = target_deg - 360;
        }
        while( target_deg <= 0 ) {
            target_deg += 360;
        }

        double curr = turn.GetPosition().GetValueAsDouble()*360;
        double target_option2 = target_deg + 360;

        double err1 = abs(target_deg - curr);
        double err2 = abs(target_option2 - curr);

        if( target_option2 < 558 && err2 < err1) {
            target_deg = target_option2;
        }

        PositionVoltage m_request = PositionVoltage{units::degree_t(target_deg)}.WithSlot(0).WithFeedForward(-1.7_V * swerve::get_chassis_speeds().omega_output);
        turn.SetControl(m_request);
    }

    void snapp_field_relative(double target_deg) {
        target_deg = target_deg - units::degree_t(swerve::get_heading()).value() + (306.85 - 180);
        snapp_robot_relative(target_deg);
    }


    double get_sixteen_encoder_value() {
        return sixteen.GetAbsolutePosition().GetValueAsDouble();
    }
    double get_seventeen_encoder_value() {
        return seventeen.GetAbsolutePosition().GetValueAsDouble();
    }

    double get_cur_target_angle () {
        return cur_target_angle;
    }

    vec2 point_to_track() {
        blue_vec2 vec = vec2{181.8067, 159.0067} * 0.0254;
        return vec;
    }

    bool wants_balls() {
        using enum state;
        if (curstate == ASHOOT || curstate == MSHOOT) {
            return true;
        }
        return false;
    }

    vec3 SOTM_shot(); 
    void handle_states() {
        using enum state;
        if (!zeroed) {
            curstate = ZEROING;
        }
        switch (curstate){
            case ATRACK:
            case ASHOOT: {
                vec3 m_SOTM_shot = SOTM_shot();
                double mpersec_version_of_flywheel_speed = m_SOTM_shot.length();
                units::angle::radian_t radianspersec_version_of_flywheel_speed {mpersec_version_of_flywheel_speed / 0.0508};
                units::turn_t rotationspermin_version_of_flywheel_speed = radianspersec_version_of_flywheel_speed * 60;
                VelocityTorqueCurrentFOC m_request {rotationspermin_version_of_flywheel_speed / 1_min };
                leader_shooter_motor.SetControl(m_request);
                

                vec2 owl_angle = m_SOTM_shot.xy();
                PositionVoltage m_request1 {owl_angle.theta()}; 
                snapp_field_relative(owl_angle.theta() * 57.2958 / 1_rad);

                vec2 hard_to_describe = m_SOTM_shot.xy(); 
                vec2 also_htd = vec2 {hard_to_describe.length(), m_SOTM_shot.z}; 
                PositionVoltage hood_angle = PositionVoltage{also_htd.theta()};
                hood_motor.SetControl(hood_angle);
            } break;

            case MSHOOT:
                //shooting sequence
                break;
            case MTRACK:
                //shooting sequence
                break;
            case RAGDOLL:
                //shooting sequence
                break;
            case ZEROING:
                if( frc::RobotBase::IsSimulation() ) {
                    hood_motor.SetPosition(0_deg);
                    turn.SetPosition(0_deg);
                    curstate = RAGDOLL;
                    zeroed = true; 
                }
                hood_motor.SetControl(DutyCycleOut{0.5}.WithIgnoreSoftwareLimits(true)); 
                hood_zeroer.calculate(hood_motor.GetStatorCurrent().GetValueAsDouble() > 40 && hood_motor.GetVelocity().GetValueAsDouble() < 0.1);
                if (hood_zeroer ) {
                    hood_motor.SetControl(DutyCycleOut{0});
                    hood_motor.SetPosition(66.0000067_deg);
                    hood_zeroer.calculate(false);
                    curstate = RAGDOLL;
                    zeroed = true; 
                }

                break;
        };
    }

    void set_to_ZEROING () {
        curstate = state::ZEROING;
    }

    void set_to_ASHOOT () {
        curstate = state::ASHOOT;
    }

    void set_to_ATRACK () {
        curstate = state::ATRACK;
    }

    void init () {
        populate_interpolating_map();
        enslaved_shooter_motor.SetControl(Follower{leader_shooter_motor.GetDeviceID(), true});
        turn.GetConfigurator().Apply(cfg::owl_config);
        enslaved_shooter_motor.GetConfigurator().Apply(cfg::shooter_slave_config);
        leader_shooter_motor.GetConfigurator().Apply(cfg::shooter_config);
        hood_motor.GetConfigurator().Apply(cfg::hood_cfg);
    }

    vec3 SOTM_shot () {
        vec2 target_point = point_to_track();
        vec2 cur_pose = swerve::get_vec2();
        vec2 cur_speed = swerve::get_chassis_speeds().xy_output; 
        vec2 err = target_point - cur_pose; 
        vec2 shotmap_shot = vec2 {units::radians_per_second_t(1_rpm * dist_and_rpm[err.length()]).value()*2*0.0254, units::radian_t{units::degree_t(dist_and_hood_angle[err.length()])}};
        vec2 twod_returnable = err.hat() * shotmap_shot.x;
        vec3 actual_returnable = vec3 {twod_returnable.x, twod_returnable.y, shotmap_shot.y};
        actual_returnable -= cur_speed;
        return actual_returnable; 
    }

    ctre::phoenix6::sim::TalonFXSimState& get_owl_sim() { return turn.GetSimState(); }
    ctre::phoenix6::sim::TalonFXSimState& get_hood_sim() { return hood_motor.GetSimState(); }
    ctre::phoenix6::sim::TalonFXSimState& get_flywheel_sim() { return leader_shooter_motor.GetSimState(); }
};
