#pragma once

class ema_filter
{
public:
    ema_filter( double alpha, double initial_value=0 );

    void calculate( double input );
    void reset( double value );
    operator double() const;

private:
    double alpha;
    double value;
};
