#include "rigid_body.h"

using namespace pba;

struct MassMoment{
	Vector moment;
	float mass;
};


RigidBodyStateData::RigidBodyStateData(){
	_initialize_default_attributes();
}

void RigidBodyStateData::_initialize_default_attributes() 
{
	DynamicalStateData::_initialize_default_attributes();
	add_attribute<Vector>("initial_positions", DSAv());
	
	add_attribute<Vector>("lever_arms", DSAv());
	_lever_arms_iter = _vec_attr.find("lever_arms");
}

void RigidBodyStateData::set_initial_position(size_t p, const Vector& pos) {
	set_vector_attribute("initial_positions", p, pos);
	// TO DO -> optimize
	init_rbd();
}

size_t RigidBodyStateData::add() {
	auto size = DynamicalStateDataBase::add();
	// TO DO -> optimize instead of recalculating everything
	init_rbd();
	return size;
}

size_t RigidBodyStateData::add(size_t n){
	auto size = DynamicalStateDataBase::add(n);
	init_rbd();
	return size;
}

void RigidBodyStateData::resize(size_t n){
	DynamicalStateDataBase::resize(n);
	init_rbd();
}

Vector RigidBodyStateData::get_vert_pos(size_t p) const {
	return angular_rotation * get_lever_arm(p) + center_of_mass;
}


Vector RigidBodyStateData::get_rotated_lever_arm(const size_t p) const{
	return angular_rotation * get_lever_arm(p);
}

void RigidBodyStateData::compute_com() {
	auto masses = get_float_attribute_span("mass");
	auto positions = get_vector_attribute_span("initial_positions");
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

void RigidBodyStateData::compute_moi(){
	for (int i = 0; i <= 2; i++){
		for (int j = 0; j <= 2; j++){
			_compute_moi(i, j);
		}
	}
	_inverse_moi = inverse(_moment_of_inertia);
}

void RigidBodyStateData::_compute_moi(int i, int j) noexcept {
	auto masses = get_float_attribute_span("mass");

	// TODO -> implement a solution with std::ranges that zips positions and masses.
	
	// A bit of a hack to get indices to loop over. 
	// Requires a TON of dynamic allocation
	//std::vector<size_t> idx(_n_particles);
	//std::iota(idx.begin(), idx.end(), 0);
	double res = 0;
	for (size_t i = 0; i < _n_particles; i++){
		auto larm = get_rotated_lever_arm(i);
		double delta = i == j ? 1.0 : 0.0;
		double mag = larm.magnitude();
		res += (double)masses[i] * (delta * mag * mag - larm[i] * larm[j]);
	}
	// double res = std::transform_reduce(
	// 	std::execution::par,
	// 	masses.begin(), masses.end(),
	// 	0.0,
	// 	std::plus<double>(), 
	// 	[this, i, j, base = masses.data()](float mi) {
	// 		size_t index = (size_t)(&mi - base);
	// 		auto larm = get_rotated_lever_arm(index);
	// 		double delta = i == j ? 1.0 : 0.0;
	// 		double mag = larm.magnitude();
	// 		return (double)mi * (delta * mag * mag - larm[i] * larm[j]);
	// 	}
	// );
	_moment_of_inertia.Set(i, j, res);
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

