#include <stdio.h>

#include "dave/vector.hpp"

int main(int argc, char** argv) {

  vec2 a{ 1, 1 };
  vec2 b = a.rotated_towards( vec2{-1,-1}, 100_deg);
  vec2 c = a.rotated_towards( vec2{-1,1}, 120_deg);

  printf( "%f, %f\n", b.x, b.y );
  printf( "%f, %f\n", c.x, c.y );

  printf( "%f %f %f\n", vec2{1, 1}.angle_to(vec2{1, 0}).value(), vec2{-1, -1}.angle_to(vec2{1, 1}).value(), vec2{2, 1}.angle_to(vec2{2, 2}).value() );

  return 0;
}
