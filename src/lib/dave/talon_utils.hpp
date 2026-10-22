#pragma once

#include <ctre/phoenix6/TalonFX.hpp>

void configure( ctre::phoenix6::hardware::TalonFX &talon, ctre::phoenix6::configs::TalonFXConfiguration &cfg, int attempts=2 );
