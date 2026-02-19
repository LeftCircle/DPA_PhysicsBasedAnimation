#include "force_library.h"


using namespace pba;



void SimpleGravityForce::compute(DynamicalStateData_sp dsd, const double dt) const {
	const size_t n = dsd->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		dsd->set_acceleration(i, dsd->get_acceleration(i) + _gravity);
	}
}

void SPHPressureForce::compute_sph(SPHData_sp sph_data, const double dt) const {
	const size_t n = sph_data->n_particles();
	const auto densities = sph_data->get_double_attribute_span("density");
	const auto rest_density = sph_data->get_uniform<double>("rest_density");
	const auto p_bar = sph_data->get_uniform<double>("p_bar");
	const auto gamma = sph_data->get_uniform<double>("gamma");
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		Vector pressure_force = _occupancy_volume->accumulate_neighbor_cells(
			i,
			sph_data->get_position(i),
			Vector(0.0, 0.0, 0.0),
			[&sph_data, &densities, p_bar, rest_density, gamma, this](size_t idx, Vector acc, const std::vector<size_t>& neighbor_indices){
				const std::span<const size_t> neighbor_indices_span(neighbor_indices);
				return acc + _get_pressure_with_uniform_h(
					idx,
					neighbor_indices_span,
					sph_data,
					densities,
					p_bar,
					rest_density,
					gamma);
			}
		);
		sph_data->set_acceleration(i, sph_data->get_acceleration(i) + pressure_force / sph_data->get_mass(i));
	}
}

Vector SPHPressureForce::_get_pressure_with_uniform_h(
		size_t i,
		const std::span<const size_t>& neighbor_indices,
		const SPHData_sp sph_data,
		const std::span<const double>& densities,
		double p_bar,
		double rest_density,
		double gamma
	) const
{
	const Vector& pos_i = sph_data->get_position(i);
	const auto dens_i = densities[i];
	const auto p_at_i = _get_pressure_at_density(dens_i, p_bar, rest_density, gamma);
	const auto d_i_sq = dens_i * dens_i;
	Vector pressure_force(0.0, 0.0, 0.0);
	for (size_t j : neighbor_indices) {
		if (j == i) continue; // skip self interactions
		const Vector& pos_j = sph_data->get_position(j);
		double distance = (pos_i - pos_j).magnitude();
		Vector grad_ab = _kernel->gradient(distance, pos_i - pos_j);
		const auto dens_j = densities[j];
		const auto p_at_j = _get_pressure_at_density(dens_j, p_bar, rest_density, gamma);
		const auto d_j_sq = dens_j * dens_j;
		pressure_force += sph_data->get_mass(j) * (p_at_i / d_i_sq + p_at_j / d_j_sq) * grad_ab;
	}
	return pressure_force;
}

double SPHPressureForce::_get_pressure_at_density(double density, double p_bar, double rest_density, double gamma) const {
	return p_bar * (pow(density / rest_density, gamma) - 1.0);
}