#include "rigid_body.h"

using namespace pba;

struct MassMoment{
	Vector moment;
	float mass;
};


void RigidBodyStateData::compute_com() {
	auto masses = get_float_attribute_span("mass");
	auto positions = get_vector_attribute_span("positions");
	// TODO -> implement a solution with std::ranges that zips positions and masses.
	MassMoment result = std::accumulate(masses.begin(), masses.end(), MassMoment(),
		[&positions, i = 0](MassMoment acc, float mass) mutable {
			acc.mass += mass;
			acc.moment += positions[i] * mass;
			i++;
			return acc;
		}
	);

	if (result.mass > 0.0)
	{
		center_of_mass = result.moment / result.mass;
	}
	else
	{
		center_of_mass = Vector(0,0,0);
	}
}

void RigidBodyStateData::compute_com_for_loop() {
	float total_mass = 0.0;
	auto masses = get_float_attribute_span("mass");
	auto positions = get_vector_attribute_span("positions");
	Vector pos_times_mass = Vector(0, 0, 0);
	for (size_t i = 0; i < _n_particles; i++){
		pos_times_mass += masses[i] * positions[i];
		total_mass += masses[i];
	}
	if (total_mass > 0.0)
	{
		center_of_mass = pos_times_mass / total_mass;
	}
	else
	{
		center_of_mass = Vector(0,0,0);
	}
}

