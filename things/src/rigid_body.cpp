#include "rigid_body.h"

using namespace pba;

struct MassMoment{
	Vector moment;
	float mass;
};


RigidBodyStateData::RigidBodyStateData(){
	add_attribute<Vector>("lever_arms", DSAv());
}

size_t RigidBodyStateData::add() {
	auto size = DynamicalStateData::add();
	// TO DO -> optimize instead of recalculating everything
	init_rbd();
	return size;
}

size_t RigidBodyStateData::add(size_t n){
	auto size = DynamicalStateData::add(n);
	init_rbd();
	return size;
}

void RigidBodyStateData::resize(size_t n){
	DynamicalStateData::resize(n);
	init_rbd();
}

void RigidBodyStateData::set_position(size_t i, const Vector& v) {
	DynamicalStateData::set_position(i, v);
	// TO DO -> optimize
	init_rbd();
}

Vector RigidBodyStateData::get_vert_pos(size_t p) {
	
}

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
		printf("Center of mass is zero!!");
	}
	_total_mass = result.mass;
}

void RigidBodyStateData::compute_lever_arms() {
	auto larm_span = get_vector_attribute_span("lever_arms");
	auto positions = get_vector_attribute_span("positions");
	const Vector& com = center_of_mass;
	// just a basic transform
	std::transform(std::execution::par, positions.begin(), positions.end(), larm_span.begin(), 
		[&com](const Vector& pos){
			return pos - com;
		});
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

