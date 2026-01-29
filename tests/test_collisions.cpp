#include "catch_amalgamated.hpp"
#include "catch_helpers.h"

#include <map>
#include <vector>

#include "dynamical_state_data.h"
#include "Vector.h"
#include "partial_solvers.h"
#include "GISolver.h"
#include "collision_object.h"

using namespace pba;


TEST_CASE( " Test Particle Plane Collision "){

    // Define a horizontal plane at y=0 with normal pointing up
    auto collision_plane_sp = create_collision_plane(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
    auto other_plane_sp = create_collision_plane(Vector(0.0, -10.0, 0.0), Vector(0.0, 1.0, 0.0));
    const double dt = 1.0;
    const Vector initial_position = Vector(0.0, 1.0, 0.0);
    const Vector updated_position = Vector(0.0, -1.0, 0.0);
    const Vector velocity = (updated_position - initial_position) / dt;

    // The collision test should return the time that the collision occurs, or -1 if no collision
    CollisionHitInfo hit_info;
    CollisionHitInfo miss_info;
    collision_plane_sp->hit(initial_position, updated_position, velocity, dt, hit_info);
    other_plane_sp->hit(initial_position, updated_position, velocity, dt, miss_info);
    
    REQUIRE(hit_info.time_of_impact == 0.5);
    REQUIRE(hit_info.position == Vector(0.0, 0.0, 0.0));
    REQUIRE(hit_info.normal == Vector(0.0, 1.0, 0.0));
    
    REQUIRE(miss_info.time_of_impact == NO_COLLISION);
}

TEST_CASE( " Test Multiple Collision Planes "){
    auto plane1 = create_collision_plane(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
    auto plane2 = create_collision_plane(Vector(0.0, 0.0, 0.0), Vector(1.0, 0.0, 0.0));
    const double dt = 10.0;
    const Vector initial_pos = Vector(2.0, 1.0, 0.0);
    const Vector initial_velocity = Vector(-1.0, -1.0, 0.0);

    auto coll_surf = create_collision_surface();
    coll_surf->add_collision_object(plane1);
    coll_surf->add_collision_object(plane2);
    
    auto collision_handler = create_collision_handler();
    collision_handler->register_collision_surface(coll_surf);
    
    DynamicalStateData_sp dsd = create_dynamical_state_data();
    dsd->add(1);
    dsd->set_position(0, initial_pos);
    dsd->set_velocity(0, initial_velocity);

    auto solver = create_advance_position_with_collisions(dsd, collision_handler);
    solver->solve(dt);

    REQUIRE(dsd->get_position(0).X() > initial_pos.X());
    REQUIRE(dsd->get_position(0).Y() > initial_pos.Y());
    REQUIRE(dsd->get_velocity(0).X() > 0.0);
    REQUIRE(dsd->get_velocity(0).Y() > 0.0);


}

TEST_CASE( " Test point on plane does not fall through"){
    auto dsd = create_dynamical_state_data();
    dsd->add(1);
    dsd->set_position(0, Vector(0.0, 0.0, 0.0));
    dsd->set_velocity(0, Vector(0.0, -1.0, 0.0));
    const double dt = 1.0;

    auto collision_plane = create_collision_plane(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
    auto collision_surface = create_collision_surface();
    collision_surface->add_collision_object(collision_plane);

    auto collision_handler = create_collision_handler();
    collision_handler->register_collision_surface(collision_surface);

    auto solver = create_advance_position_with_collisions(dsd, collision_handler);
    solver->solve(dt);

    REQUIRE(dsd->get_position(0) == Vector(0.0, 1.0, 0.0));
}

TEST_CASE( " Test no infinite loop for static particle on plane"){
    auto dsd = create_dynamical_state_data();
    dsd->add(1);
    dsd->set_position(0, Vector(0.0, 0.0, 0.0));
    dsd->set_velocity(0, Vector(0.0, 0.0, 0.0));
    const double dt = 1.0;

    auto collision_plane = create_collision_plane(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
    auto collision_surface = create_collision_surface();
    collision_surface->add_collision_object(collision_plane);

    auto collision_handler = create_collision_handler();
    collision_handler->register_collision_surface(collision_surface);

    auto solver = create_advance_position_with_collisions(dsd, collision_handler);
    solver->solve(dt);

    REQUIRE(dsd->get_position(0) == Vector(0.0, 0.0, 0.0));
}

TEST_CASE(" Test no fall through for cr of 0"){
    auto dsd = create_dynamical_state_data();
    dsd->add(1);
    dsd->set_position(0, Vector(0.0, 0.0, 0.0));
    dsd->set_velocity(0, Vector(0.0, -1.0, 0.0));
    const double dt = 1.0;

    auto collision_plane = create_collision_plane(Vector(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0));
    auto collision_surface = create_collision_surface();
    collision_surface->add_collision_object(collision_plane);
    collision_surface->set_restitution(0.0);

    auto collision_handler = create_collision_handler();
    collision_handler->register_collision_surface(collision_surface);

    auto solver = create_advance_position_with_collisions(dsd, collision_handler);
    solver->solve(dt);

    REQUIRE(dsd->get_position(0) == Vector(0.0, 0.0, 0.0));
    REQUIRE(dsd->get_velocity(0) == Vector(0.0, 0.0, 0.0));
}

TEST_CASE("Particle resting on plane does not bounce due to gravity"){
    // If a particle starts and is resting on a plane and the velocity is zero,
    // If we run the force solver to apply gravity, then the collision handling, 
    // the particle should not bounce. 

    REQUIRE(false);
}