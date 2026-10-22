#pragma once

#include "config.hpp"

#include <ctre/phoenix6/CANdle.hpp>

#include "swerve.hpp"

#include "dave/morse_code_performer.hpp"

namespace leds {

    using ctre::phoenix6::signals::RGBWColor;
    using namespace ctre::phoenix6::controls;

    morse_code_performer morse_performer;

    ctre::phoenix6::hardware::CANdle candle{ cfg::can_candle, cfg::canivore };

    RainbowAnimation rainbow{ 8, 24 };
    FireAnimation fire{ 8, 24 };
    SolidColor solid = SolidColor{ 8, 24 }
        .WithColor( RGBWColor{ 0, 200, 0 } );
    SolidColor off = SolidColor{ 8, 24 }
        .WithColor( RGBWColor{ 0, 0, 0 } );

    SolidColor solid8 = SolidColor{ 0, 7 }
        .WithColor( RGBWColor{ 0, 17, 0 } );
    SolidColor off8 = SolidColor{ 0, 7 }
        .WithColor( RGBWColor{ 0, 1, 0 } );

    void init() {
        candle.GetConfigurator().Apply( ctre::phoenix6::configs::CANdleConfiguration{}
            .WithLED( ctre::phoenix6::configs::LEDConfigs{}
                .WithStripType( ctre::phoenix6::signals::StripTypeValue::GRB )
                .WithLossOfSignalBehavior( ctre::phoenix6::signals::LossOfSignalBehaviorValue::DisableLEDs )
            )
            .WithCANdleFeatures( ctre::phoenix6::configs::CANdleFeaturesConfigs{}
            )
        );

        morse_performer.load_file( "beemovie.txt" );
        morse_performer.begin();
    }

    void set_outputs() {
        bool morse_code_on = morse_performer.get_state();

        if( swerve::collided( 500_ms ) ) {
            candle.SetControl( fire );
        } else {
            candle.SetControl( EmptyAnimation{0} );
            candle.SetControl( off );
        }

        candle.SetControl( morse_code_on ? solid8 : off8 );
        frc::SmartDashboard::PutBoolean( "morse code test", morse_code_on );
    }

}
