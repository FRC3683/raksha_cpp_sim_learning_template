#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <initializer_list>

struct vec2 {
    double x, y;

    static const vec2 zero;

    vec2();
    vec2( double x, double y );
    vec2( double length, units::radian_t theta );
    vec2( double length, units::degree_t theta );
    vec2( double length, frc::Rotation2d theta );
    vec2( frc::Translation2d t );

    units::radian_t theta() const;
    double length_squared() const;
    double length() const;
    double inv_length() const;
    vec2 hat() const;
    units::radian_t angle_to( const vec2 &other ) const;
    vec2 rotated_by( units::radian_t beta ) const;
    vec2 rotated_towards( const vec2 &other, units::radian_t max_delta ) const;
    vec2 moved_towards( const vec2 &other, const double step ) const;
    vec2 clamped( const double max_length ) const;
    vec2 deadbanded( const double deadzone, const double max_value ) const;
    vec2 projected_onto( const vec2 &onto ) const;
    double dot( const vec2 &other ) const;
    double cross( const vec2 &other ) const;
    vec2 closest_from( std::initializer_list<vec2> options ) const;

    frc::Translation2d t() const;

    vec2& operator+=( const vec2 &other );
    vec2& operator-=( const vec2 &other );
    vec2& operator/=( const double f );
    vec2& operator*=( const double f );
};

vec2 operator+( const vec2 &v1, const vec2 &v2 );
vec2 operator-( const vec2 &v1, const vec2 &v2 );
vec2 operator*( const vec2 &v, const double f );
vec2 operator*( const double f, const vec2 &v );
vec2 operator/( const vec2 &v, const double f );
vec2 operator-( const vec2& v );

vec2 lerp( const vec2 &v1, const vec2 &v2, const double alpha );


struct vec3 {
    double x, y, z;

    static const vec3 zero;

    vec3();
    vec3( double x, double y, double z );
    vec3( frc::Translation3d t );
    vec3( double length, frc::Rotation3d theta );
    vec3( vec2 xy, double z );
    vec3( vec2 xy );

    vec2 xy() const;
    frc::Rotation3d theta() const;
    double length_squared() const;
    double length() const;
    double inv_length() const;
    vec3 hat() const;
    vec3 moved_towards( const vec3 &other, const double step ) const;
    vec3 clamped( const double max_length ) const;
    vec3 deadbanded( const double deadzone, const double max_value ) const;
    vec3 projected_onto( const vec3 &onto ) const;
    double dot( const vec3 &other ) const;
    vec3 cross( const vec3 &other ) const;
    vec3 rotated_around_x( units::degree_t theta ) const;
    vec3 rotated_around_y( units::degree_t theta ) const;
    vec3 rotated_around_z( units::degree_t theta ) const;

    frc::Translation3d t() const;

    vec3& operator+=( const vec3 &other );
    vec3& operator-=( const vec3 &other );
    vec3& operator/=( const double f );
    vec3& operator*=( const double f );
};

vec3 operator+( const vec3 &v1, const vec3 &v2 );
vec3 operator-( const vec3 &v1, const vec3 &v2 );
vec3 operator*( const vec3 &v, const double f );
vec3 operator*( const double f, const vec3 &v );
vec3 operator/( const vec3 &v, const double f );
vec3 operator-( const vec3& v );

vec3 lerp( const vec3 &v1, const vec3 &v2, const double alpha );
