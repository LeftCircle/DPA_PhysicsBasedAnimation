#include "catch_helpers.h"
#include "boids_acceleration.h"
#include "boid_solvers.h"
#include "boids_state_data.h"

using namespace pba;

TEST_CASE("test boid forces"){
    /*
        <-o

    <-o   x   <-o
        
        <-o
    */
   // The x boid above should have a centering of zero
   // avoidance of zero, and vel match in <- direction
   BoidStateData dsd;
   dsd.add(5);
   dsd.set_position(0, Vector(-1, 0, 0));
   dsd.set_position(1, Vector(0, 1, 0));
   dsd.set_position(2, Vector(1, 0, 0));
   dsd.set_position(3, Vector(0, -1, 0));
   dsd.set_position(4, Vector(0, 0, 0));


}