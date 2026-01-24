#include "catch_amalgamated.hpp"

#include <map>
#include <vector>

#include "dynamical_state_data.h"
#include "Vector.h"

using namespace pba;

TEST_CASE( "DSData only returns iterators to DSAttribute arrays"){
	DynamicalStateData_sp dsd = create_dynamical_state_data();

	// We don't want to expose the DSAttributes directly, because then a 
	// user might add to the attribute without expanding all the other attributes.
	// Therefore, we only give access to the iterators of the DSAttributes.

	auto& pos_iter = dsd->get_positions();
	auto& vel_iter = dsd->get_velocities();
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

TEST_CASE( " ensure that you cannot add a particle to only a single DSAttribute"){
	// Adding a particle should expand all attributes, not just one. 
	// TODO -> This might require a bit of refactoring. We should never be able
	// to actually get the DSAttribute objects outside of the DynamicalStateData class.
	// We should also never be able to access the array within the DSAttribute directly.
	// This will ensure that we can control the particle count properly.

	// The only time we can access the DSAttribute or its data directly is through 
	// const accessors or through the DynamicalStateData class methods.
	REQUIRE(false);
}
