#include "dave/morse_code_performer.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/Filesystem.h>
#include <fstream>
#include <sstream>
#include <cctype>

// Letters mapped from index 0 (A) to index 25 (Z)
const static std::string MORSE_LETTERS[] = {
    ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",    // A - I
    ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.",  // J - R
    "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."          // S - Z
};

// Digits mapped from index 0 ('0') to index 9 ('9')
const static std::string MORSE_DIGITS[] = {
    "-----", ".----", "..---", "...--", "....-", // 0 - 4
    ".....", "-....", "--...", "---..", "----."  // 5 - 9
};

morse_code_performer::morse_code_performer( units::second_t dit_time )
  : dit_time{dit_time}, dah_time{dit_time*3}, space_time{dit_time*5}
{}

void morse_code_performer::load( std::string morse_code_text ) {
    morse_string = morse_code_text;
}

int morse_code_performer::load_file( std::string fname ) {
    std::string deploy_dir = frc::filesystem::GetDeployDirectory();
    std::string file_path = deploy_dir + "/" + fname;

    std::ifstream file{ file_path };
    if (!file.is_open()) {
        return -1;
    }

    std::stringstream buffer;
    buffer << file.rdbuf(); // Read the file buffer into the stringstream
    morse_string = buffer.str(); // Convert stream to string

    frc::SmartDashboard::PutNumber( "string length", morse_string.length() );

    return 0;
}

void morse_code_performer::begin() {
    string_read_head = 0;
    letter_read_head = 0;
    timestamp = frc::Timer::GetFPGATimestamp();
}

bool morse_code_performer::get_state() {
    units::second_t now = frc::Timer::GetFPGATimestamp();
    units::second_t delta = now - timestamp;

    char current;
    while( true ) {
        if( string_read_head >= morse_string.length() ) {
            begin(); // loop
            return false;
        }
        current = std::toupper( morse_string[string_read_head] );
        if( std::isalpha( current ) || std::isdigit( current ) || current == ' ' ) {
            break;
        }
        ++string_read_head;
    }

    std::string morse;
    if( std::isalpha( current ) ) {
        morse = MORSE_LETTERS[current - 'A'] + "  ";
    } else if( std::isdigit( current ) ) {
        morse = MORSE_DIGITS[current - '0'] + "  ";
    } else {
        morse = "    ";
    }

    char c = morse[letter_read_head];
    units::second_t char_time = (c == '-') ? dah_time : dit_time;
    units::second_t advance_time = char_time + dit_time;

    if( delta < char_time ) {
        return c != ' ';
    } else if( delta < advance_time ) {
        return false;
    } else {
        ++letter_read_head;
        timestamp = now;
        if( letter_read_head >= morse.length() ) {
            letter_read_head = 0;
            ++string_read_head;
        }
    }
    return false;
}
