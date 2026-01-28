#include "catch_amalgamated.hpp"

#include <map>
#include <vector>

#include "dynamical_state_data.h"
#include "Vector.h"

using namespace pba;

TEST_CASE( "Benchmark iterators vs accessors"){
	// The goal of this benchmark is to see if using iterators is faster than using accessors
	// for accessing and updating particle data.
	DynamicalStateData_sp dsd = create_dynamical_state_data();

	constexpr size_t N = 1000000;
	dsd->add(N);

	// get iterators once
	//auto pos_it = dsd->get_pos_begin_iter();
	//auto vel_it = dsd->get_vel_cbegin_iter();
	//auto pos_end = dsd->get_pos_end_iter();

	constexpr float dt = 0.01f;
	
	// Benchmark showed that the versions were basically identical, so just
	// keeping and maintaining the accessor version.
	// BENCHMARK("Using iterators to update positions"){
	// 	auto pos_it = dsd->get_pos_begin_iter();
	// 	auto vel_it = dsd->get_vel_cbegin_iter();
	// 	auto pos_end = dsd->get_pos_end_iter();
	// 	for( ; pos_it != pos_end; ++pos_it, ++vel_it ){
	// 		*pos_it += (*vel_it) * dt;
	// 	}
	// 	return *pos_it;
	// };

	// BENCHMARK("Using accessors to update positions"){
	// 	const size_t n = dsd->n_particles();
	// 	for( size_t i=0; i<n; i++ ){
	// 		const Vector& pos = dsd->get_position(i);
	// 		const Vector& vel = dsd->get_velocity(i);
	// 		dsd->set_position(i, pos + vel * dt);
	// 	}
	// 	return dsd->get_position(n-1);
	// };
	REQUIRE(dsd->n_particles() == N);
}


TEST_CASE( "Does dynamical state data have default attributes"){
	DynamicalStateData_sp dsd = create_dynamical_state_data();

	REQUIRE(dsd->has_vector_attribute("positions"));
	REQUIRE(dsd->has_vector_attribute("velocities"));
	REQUIRE(dsd->has_vector_attribute("accelleration"));
	REQUIRE(dsd->has_float_attribute("mass"));
	REQUIRE(dsd->has_color_attribute("color"));
}


TEST_CASE( " Test get positions and velocities after resize"){
	// Confirm we don't have to recalculate the iterators after resizing the attributes
	// Also check to see if we have to recalculate the iterators after adding 
	// a new DSAttribute
	DynamicalStateData_sp dsd = create_dynamical_state_data();
	const size_t n1 = 10;
	dsd->add(n1);
	const Vector pos1(1.0, 2.0, 3.0);
	const Vector pos2(4.0, 5.0, 6.0);
	for (size_t i=0; i<n1; i++){
		dsd->set_position(i, pos1);
	}

	REQUIRE(dsd->get_position(5) == pos1);

	// Let's try adding a new vector attribute
	dsd->add_attribute<Vector>("test_vec_attr", DSAv("test_vec_attr", Vector(0.0f, 0.0f, 0.0f)));

	const size_t n2 = 20;
	dsd->add(n2 - n1);
	for (size_t i=0; i<n2; i++){
		dsd->set_position(i, pos2);
	}

	REQUIRE(dsd->get_position(15) == pos2);
}

TEST_CASE( " Test get attribute spans "){
	DynamicalStateData_sp dsd = create_dynamical_state_data();
	const size_t n = 100;
	dsd->add(n);

	auto pos_span = dsd->get_vector_attribute_span("positions");
	REQUIRE(pos_span.size() == n);

	for (size_t i=0; i<n; i++){
		REQUIRE(pos_span[i] == Vector(0.0f, 0.0f, 0.0f));
	}

	const Vector new_pos(1.0f, 2.0f, 3.0f);
	for (size_t i=0; i<n; i++){
		pos_span[i] = new_pos;
	}

	auto pos_span_const = dsd->get_vector_attribute_span("positions");
	for (size_t i=0; i<n; i++){
		REQUIRE(pos_span_const[i] == new_pos);
	}
}


