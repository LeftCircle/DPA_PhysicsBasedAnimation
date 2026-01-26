#include "catch_amalgamated.hpp"

#include <map>
#include <vector>

#include "dynamical_state_data.h"
#include "Vector.h"
#include "partial_solvers.h"
#include "GISolver.h"

using namespace pba;


TEST_CASE( " Test Particle Plane Collision "){

    // Define a horizontal plane at y=0 with normal pointing up
    auto collision_plane_sp = create_collision_plane(Vector(0.0f, 0.0f, 0.0f), Vector(0.0f, 1.0f, 0.0f));
    auto other_plane_sp = create_collision_plane(Vector(0.0f, -10.0f, 0.0f), Vector(0.0f, 1.0f, 0.0f));
    const double dt = 1.0;
    const Vector initial_position = Vector(0.0, 1.0, 0.0);
    const Vector updated_position = Vector(0.0, -1.0, 0.0);
    const Vector velocity = (updated_position - initial_position) / dt;

    // The collision test should return the time that the collision occurs, or -1 if no collision
    REQUIRE(collision_plane_sp->hit(initial_position, updated_position, velocity, dt) == 0.5);
    REQUIRE(other_plane_sp->hit(initial_position, updated_position, velocity, dt) == -1.0);
}