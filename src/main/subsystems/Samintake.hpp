
#pragma once
#include "shooter.hpp"
namespace Samintake {
    bool zeroed;
    bool deployed;
    enum class Intention {
        succc,
        idle,
        vomit
    };

    enum class State {
        zero,
        deploy,
        idle,
        succc,
        vomit, 
        jostle
    };

    State curstate = State::zero;
    Intention curintention = Intention::idle;

    TalonFX pivot {0, "canivore"} ;
    TalonFX master_sucker {0, "canivore"} ;
    TalonFX slave_sucker {0, "canivore"};
    debouncer pivot_zeroer = debouncer::rising_edge(300_ms, false);


    units::degree_t get_position() {
        return pivot.GetPosition().GetValue();
    }

    void handle_intentions () {
        using enum Intention; 
        switch (curintention) {
            case succc: {
                curstate = State::succc;
            } break;
            
            case idle: {
                if (shooter::wants_balls()) {
                    curstate = State::jostle;
                }
                else {
                    curstate = State::idle;
                }
            } break; 

            case vomit: {
                curstate = State::vomit;
            }break;
        };  
    }

    void handle_states () {
        handle_intentions();
        if (!zeroed) {
            curstate = State::zero;
            zeroed = true;
        }

        if (zeroed && !deployed){
            curstate = State::deploy;
            deployed = true;
        }

        using enum State;
        PositionVoltage all_the_way_down { 0_deg };
        DutyCycleOut stop {0};
        //this will clamp to -1 btw
        DutyCycleOut vomit_max {-67};
        DutyCycleOut succc_max {67};
        switch (curstate) {
            case zero: {
                pivot.SetControl(DutyCycleOut{0.5}.WithIgnoreSoftwareLimits(true)); 
                master_sucker.SetControl(vomit_max);
                    pivot_zeroer.calculate(pivot.GetStatorCurrent().GetValueAsDouble() > 40 && pivot.GetVelocity().GetValueAsDouble() < 0.05);
                    if (pivot_zeroer) {
                        pivot.SetControl(DutyCycleOut{0});
                        pivot.SetPosition(0.0000000000000000000067_deg);
                        pivot_zeroer.calculate(false);
                        curintention = Intention::idle;
                        zeroed = true;
                        deployed = true;
                    }
                }           
            break;
                    
            case deploy: {
                pivot.SetControl(all_the_way_down);
                master_sucker.SetControl(vomit_max);
                deployed = true; 
                if (3 > pivot.GetPosition().GetValueAsDouble()*360 && pivot.GetPosition().GetValueAsDouble()*360 > -3) {
                    master_sucker.SetControl(stop);
                }
            }
            break;

            case idle: {
                pivot.SetControl(all_the_way_down);
                master_sucker.SetControl(stop);
            }
            break;

            case succc: {
                pivot.SetControl(all_the_way_down);
                master_sucker.SetControl(succc_max);
            }
            break;

            case vomit: {
                pivot.SetControl(all_the_way_down);
                master_sucker.SetControl(vomit_max);
            } 

            break;

            case jostle: {
                PositionTorqueCurrentFOC jostler {45.0067_deg};
                jostler.WithSlot(1);
                pivot.SetControl(jostler);
                master_sucker.SetControl(stop);
            }
        }
    } 

    void init () {
        slave_sucker.SetControl(Follower{master_sucker.GetDeviceID(), false});
    }

    void set_state (State state) {
        curstate = state; 
    }
    
    void set_intention (Intention intention) {
        curintention = intention; 
    }

    ctre::phoenix6::sim::TalonFXSimState& get_pivot_sim() { return pivot.GetSimState(); }

}