#include "dave/vector.hpp"

#include "dave/math.hpp"

const vec2 vec2::zero{ 0, 0 };
const vec3 vec3::zero{ 0, 0, 0 };

vec2::vec2()
    : x{ 0 }, y{ 0 } {}

vec2::vec2( double x, double y ) 
    : x{ x }, y{ y } {}

vec2::vec2( double length, units::radian_t theta )
    : x{ length == 0 ? 0 : length * cos( theta ) }, y{ length == 0 ? 0 : length * sin( theta ) } {}

vec2::vec2( double length, units::degree_t theta )
    : vec2{ length, units::radian_t{ theta } } {}

vec2::vec2( double length, frc::Rotation2d theta )
    : vec2{ length, theta.Radians() } {}

vec2::vec2( frc::Translation2d t )
    : x{ t.X().value() }, y{ t.Y().value() } {}

units::radian_t vec2::theta() const {
    return arctan( y, x );
}

double vec2::length_squared() const {
    return (x * x) + (y * y);
}

double vec2::length() const {
    return std::hypot( x, y );
}

double vec2::inv_length() const {
    return 1.0 / std::sqrt( length_squared() );
}

vec2 vec2::hat() const {
    const double length = this->length();
    if( length <= 0.00000001 ) {
        return vec2::zero;
    }
    return *this / length;
}

units::radian_t vec2::angle_to( const vec2 &other ) const {
    return arctan( (x * other.y) - (y * other.x), (x * other.x) + (y * other.y) );
}

vec2 vec2::rotated_by( units::radian_t beta ) const {
    if( length_squared() < 0.00000000001 ) {
        return vec2::zero;
    }
    const double sin_beta = sin( beta );
    const double cos_beta = cos( beta );
    return vec2{ cos_beta * x - sin_beta * y,
                 sin_beta * x + cos_beta * y };
}

vec2 vec2::rotated_towards( const vec2 &other, units::radian_t max_delta ) const {
    double l = length();
    double doot = this->dot( other );
    double cos_beta = doot * other.inv_length() / l;
    units::radian_t beta = arccos( cos_beta );
    if( units::math::abs( beta ) <= units::math::abs( max_delta ) ) {
        return vec2{ l, other.theta() };
    }
    double direction = cross( other );
    if( direction == 0 ) {
        direction = 1;
    }
    return vec2{ l, theta() + ( max_delta * signum( direction ) ) };
}

vec2 vec2::moved_towards( const vec2 &other, const double step ) const {
    const vec2 delta = (other - *this);
    if( delta.length_squared() <= (step * step) ) {
        return other;
    }
    return *this + ( delta.hat() * step );
}

vec2 vec2::clamped( const double max_length ) const {
    if( length_squared() <= max_length * max_length ) {
        return *this;
    }
    return hat() * max_length;
}

vec2 vec2::deadbanded( const double deadzone, const double max_value ) const {
    if( length_squared() < deadzone * deadzone ) {
        return vec2::zero;
    }
    return hat() * remap( length(), deadzone, max_value, 0, max_value );
}

vec2 vec2::projected_onto( const vec2 &onto ) const {
    vec2 onto_hat = onto.hat();
    return onto_hat * this->dot( onto_hat );
}

double vec2::dot( const vec2 &other ) const {
    return ( x * other.x ) + ( y * other.y );
}

double vec2::cross( const vec2 &other ) const {
    return ( x * other.y ) - ( y * other.x );
}

vec2 vec2::closest_from( std::initializer_list<vec2> options ) const {
    vec2 closest;
    double closest_dist = -1;
    for( auto &vec : options ) {
        double dist = ( *this - vec ).length();
        if( closest_dist == -1 || dist < closest_dist ) {
            closest_dist = dist;
            closest = vec;
        }
    }
    return closest;
}



frc::Translation2d vec2::t() const {
    return frc::Translation2d{ units::meter_t{ x }, units::meter_t{ y } };
}

vec2 operator+( const vec2 &v1, const vec2 &v2 ) {
    return vec2{ v1.x + v2.x, v1.y + v2.y };
}

vec2 operator-( const vec2 &v1, const vec2 &v2 ) {
    return vec2{ v1.x - v2.x, v1.y - v2.y };
}

vec2 operator*( const vec2 &v, const double f ) {
    return vec2{ v.x * f, v.y * f };
}

vec2 operator*( const double f, const vec2 &v ) {
    return vec2{ v.x * f, v.y * f };
}

vec2 operator/( const vec2 &v, const double f ) {
    return vec2{ v.x / f, v.y / f };
}

vec2& vec2::operator+=( const vec2 &other ) {
    x += other.x;
    y += other.y;
    return *this;
}

vec2& vec2::operator-=( const vec2 &other ) {
    x -= other.x;
    y -= other.y;
    return *this;
}

vec2& vec2::operator/=( const double f ) {
    x /= f;
    y /= f;
    return *this;
}

vec2& vec2::operator*=( const double f ) {
    x *= f;
    y *= f;
    return *this;
}

vec2 operator-( const vec2& v ) {
    return vec2{ -v.x, -v.y };
}

vec2 lerp( const vec2 &v1, const vec2 &v2, const double alpha ) {
    if( alpha <= 0 ) {
        return v1;
    }
    if( alpha >= 1 ) {
        return v2;
    }
    return (v2 - v1) * alpha + v1;
}

// VECTOR 3 3 3 3
// VECTOR 3 3 3 3 3 3 3 3 3 3 3 3
// VECTOR 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3
// VECTOR 3 3 3 3 3 3 3 3 3 3 3 3
// VECTOR 3 3 3 3

vec3::vec3() : x{0}, y{0}, z{0} {}
vec3::vec3( double x, double y, double z ) : x{ x }, y{ y }, z{ z } {}
vec3::vec3( frc::Translation3d t ) : x{ t.X().value() }, y{ t.Y().value() }, z{ t.Z().value() } {}
vec3::vec3( double length, frc::Rotation3d theta ) : vec3{ frc::Translation3d{ units::meter_t{ length }, theta } } {}
vec3::vec3( vec2 xy, double z ) : x{ xy.x }, y{ xy.y }, z{z} {}
vec3::vec3( vec2 xy ) : x{ xy.x }, y{ xy.y }, z{0} {}

vec2 vec3::xy() const {
    return vec2{ x, y };
}

frc::Rotation3d vec3::theta() const {
    units::radian_t yaw = vec2{ x, y }.theta();
    // positive pitch is down
    units::radian_t pitch = vec2{ -z, std::hypot( x, y ) }.theta();
    // roll is always 0
    return frc::Rotation3d{ 0_rad, pitch, yaw };
}

double vec3::length_squared() const {
    return (x * x) + (y * y) + (z * z);
}

double vec3::length() const {
    return std::sqrt( length_squared() );
}

double vec3::inv_length() const {
    return 1.0 / length();
}

vec3 vec3::hat() const {
    const double length = this->length();
    if( length <= 0.00000001 ) {
        return vec3::zero;
    }
    return *this / length;
}

vec3 vec3::moved_towards( const vec3 &other, const double step ) const {
    const vec3 delta = (other - *this);
    if( delta.length_squared() <= (step * step) ) {
        return other;
    }
    return *this + ( delta.hat() * step );
}

vec3 vec3::clamped( const double max_length ) const {
    if( length_squared() <= max_length * max_length ) {
        return *this;
    }
    return hat() * max_length;
}

vec3 vec3::deadbanded( const double deadzone, const double max_value ) const {
    if( length_squared() < deadzone * deadzone ) {
        return vec3::zero;
    }
    return hat() * remap( length(), deadzone, max_value, 0, max_value );
}

vec3 vec3::projected_onto( const vec3 &onto ) const {
    return onto * this->dot( onto ) / onto.length_squared();
}

double vec3::dot( const vec3 &other ) const {
    return (x * other.x) + (y * other.y) + (z * other.z);
}

vec3 vec3::cross( const vec3 &other ) const {
    return vec3{ (y * other.z) - (z * other.y), (z * other.x) - (x * other.z), (x * other.y) - (y * other.x) };
}


vec3 vec3::rotated_around_x( units::degree_t theta ) const {
    auto sin_theta = units::math::sin( theta );
    auto cos_theta = units::math::cos( theta );
    return vec3{
        x, 
        y * cos_theta - z * sin_theta, 
        y *  sin_theta + z * cos_theta
    };
}

vec3 vec3::rotated_around_y( units::degree_t theta ) const {
    auto sin_theta = units::math::sin( theta );
    auto cos_theta = units::math::cos( theta );
    return vec3{
        x * cos_theta + z * sin_theta,
        y,
        -x * sin_theta + z * cos_theta
    };
}

vec3 vec3::rotated_around_z( units::degree_t theta ) const {
    auto sin_theta = units::math::sin( theta );
    auto cos_theta = units::math::cos( theta );
    return vec3{
        x * cos_theta - y * sin_theta,
        x * sin_theta + y * cos_theta,
        z
    };
}


frc::Translation3d vec3::t() const {
    return frc::Translation3d{ units::meter_t{x}, units::meter_t{y}, units::meter_t{z} };
}

vec3& vec3::operator+=( const vec3 &other ) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

vec3& vec3::operator-=( const vec3 &other ) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

vec3& vec3::operator/=( const double f ) {
    x /= f;
    y /= f;
    z /= f;
    return *this;
}

vec3& vec3::operator*=( const double f ) {
    x *= f;
    y *= f;
    z *= f;
    return *this;
}


vec3 operator+( const vec3 &v1, const vec3 &v2 ) {
    return vec3{ v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}

vec3 operator-( const vec3 &v1, const vec3 &v2 ) {
    return vec3{ v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

vec3 operator*( const vec3 &v, const double f ) {
    return vec3{ v.x * f, v.y * f, v.z * f };
}

vec3 operator*( const double f, const vec3 &v ) {
    return vec3{ v.x * f, v.y * f, v.z * f };
}

vec3 operator/( const vec3 &v, const double f ) {
    return vec3{ v.x / f, v.y / f, v.z / f };
}

vec3 operator-( const vec3& v ) {
    return vec3{ -v.x, -v.y, -v.z };
}


vec3 lerp( const vec3 &v1, const vec3 &v2, const double alpha ) {
    if( alpha <= 0 ) {
        return v1;
    }
    if( alpha >= 1 ) {
        return v2;
    }
    return (v2 - v1) * alpha + v1;
}


