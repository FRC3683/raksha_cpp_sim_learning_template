#pragma once

#include <initializer_list>
#include <string>

#include "dave/schedulable.hpp"
#include "dave/field.hpp"

struct auton {
    std::string dashboard_name = "none";
    schedulable *to_schedule;
    std::optional<red_pose2d> preferred_starting_pose = std::nullopt;
};

class auto_selector {

public:
    auto_selector( std::initializer_list<auton> autons );
    void disabled_periodic();
    auton& get_selected();

private:
    void select( int i );

private:
    static constexpr int banksize = 9;
    static constexpr int select_buttons[ banksize ] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    static constexpr int shift_btn = 10;

    auton autons[ banksize * 2 ];
    int selected = 0;

};
