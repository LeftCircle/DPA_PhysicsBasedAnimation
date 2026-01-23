#include "catch_amalgamated.hpp"

#include <map>
#include <vector>

#include "dynamical_state_data.h"
#include "Vector.h"

using namespace pba;

TEST_CASE( "cache default state data on creation"){
	// Create dynamical state data and ensure that the default data,
	// positions, velocities, acceleration, and masss are cached. 

	// factory func to create a shared pointer to a dynamical state data object. 
	DynamicalStateData_sp dsd = create_dynamical_state_data();

	DSAttribute<pba::Vector>& pos_attr_a = dsd->get_positions();
	DSAv& pos_attr_b = dsd->get_attribute<Vector>("positions");
	REQUIRE(&pos_attr_a == &pos_attr_b);
}

TEST_CASE( "Does dynamical state data have default attributes"){
	DynamicalStateData_sp dsd = create_dynamical_state_data();

	REQUIRE(dsd->has_vector_attribute("positions"));
	REQUIRE(dsd->has_vector_attribute("velocities"));
	REQUIRE(dsd->has_vector_attribute("accelleration"));
	REQUIRE(dsd->has_float_attribute("mass"));
	REQUIRE(dsd->has_color_attribute("color"));
}

TEST_CASE( "get iterator from map "){
	// If we have something like lifetime, we don't want to have to grab the map and find 
	// the lifetime iterator each time we look it up. If we can nab the dsattribribute once 
	// at the start of compute, it will save a lot of time
	DynamicalStateData_sp dsd = create_dynamical_state_data();

	dsd->add_attribute("lifetime", DSAf("lifetime", 1.0f));
	dsd->add();
	DSAf& lifetime_dsa = dsd->get_attribute<float>("lifetime");
	REQUIRE(lifetime_dsa.get(0) == 1.0);
}


TEST_CASE( " Test get positions and velocities after resize"){
	// Confirm we don't have to recalculate the iterators after resizing the attributes
	// Also check to see if we have to recalculate the iterators after adding 
	// a new DSAttribute
	REQUIRE(false);
}
