#pragma once

#include <string>

class preference {
public:
    preference( std::string key, double default_value );
    operator double();

private:
    bool initialized = false;
    std::string key;
    double default_value;
};


class dash_toggle {
public:
    dash_toggle( std::string key, bool default_value=true );
    operator bool();

private:
    bool initialized = false;
    std::string key;
    bool default_value;
};
