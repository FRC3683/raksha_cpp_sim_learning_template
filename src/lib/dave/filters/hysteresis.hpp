#pragma once

class hysteresis {
public:
    hysteresis( double tight_threshold, double loose_threshold, bool initial_value=false );
    void calculate( double measurement );

    operator bool() const;

private:
    double tight_threshold, loose_threshold;
    bool value;
};
