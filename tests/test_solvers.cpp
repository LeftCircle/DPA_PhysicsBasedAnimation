#include "catch_amalgamated.hpp"

#include <map>
#include <vector>

#include "dynamical_state_data.h"
#include "Vector.h"
#include "partial_solvers.h"
#include "GISolver.h"

using namespace pba;


TEST_CASE( "Test advance position "){
    auto dsd = create_dynamical_state_data();
    dsd->add(10);
    dsd->set_velocity(0, Vector(1.0f, 0.0f, 0.0f));
    GISolver_sp GIAdvancePosition = std::make_shared<PartialSolverAdvancePosition>(dsd);
    GIAdvancePosition->init();
    const double dt = 0.1;
    GIAdvancePosition->solve(dt);

    REQUIRE( dsd->get_position(0) == Vector(0.1f, 0.0f, 0.0f) );

}

TEST_CASE(" Handle Collision With Static Plane") {
    // Test that a particle being moved through a plane returns collision info
    
    
}