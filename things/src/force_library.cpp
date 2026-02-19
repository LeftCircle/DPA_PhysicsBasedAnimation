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


void SPHViscocityForce::compute_sph(SPHData_sp sph_data, const double dt) const {
	const size_t n = sph_data->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		Vector viscocity_force = _occupancy_volume->accumulate_neighbor_cells(
			i,
			sph_data->get_position(i),
			Vector(0.0, 0.0, 0.0),
			[&sph_data, this](size_t idx, Vector acc, const std::vector<size_t>& neighbor_indices){
				return acc + _compute_viscocity_force_for_neighbors(idx, neighbor_indices, sph_data);
			}
		);
		sph_data->set_acceleration(i, sph_data->get_acceleration(i) + viscocity_force / sph_data->get_mass(i));
	}
}

// This is a tad bit of a mess. The main issue is keeping all of the uniforms tucked into 
// the SPHData uniforms, which leads to the parameters being pulled before the accumulate
// to avoid doing the map lookup each time. 
// A better appraoch would be to just have these uniforms be defined in the SPHDSD class as
// members instead of in the uniform map. 
Vector SPHViscocityForce::_compute_viscocity_force_for_neighbors(size_t i, const std::span<const size_t>& neighbor_indices, const SPHData_sp sph_data) const {
	const auto densities = sph_data->get_double_attribute_span("density");
	const auto one_over_rd = 1.0 / sph_data->get_uniform<double>("rest_density");
	const auto p_bar = sph_data->get_uniform<double>("p_bar");
	const auto gamma = sph_data->get_uniform<double>("gamma");
	const auto rp = sph_data->get_uniform<double>("rest_pressure");
	double h = sph_data->get_uniform<double>("smoothing_length");
	double eps = sph_data->get_uniform<double>("viscocity_epsilon");
	double alpha = sph_data->get_uniform<double>("viscocity_alpha");
	double beta = sph_data->get_uniform<double>("viscocity_beta");
	return std::accumulate(neighbor_indices.begin(), neighbor_indices.end(), Vector(0.0, 0.0, 0.0), 
		[&sph_data, i, densities, one_over_rd, p_bar, gamma, rp, h, eps, alpha, beta, this](Vector acc, size_t j){
			if (j == i) [[unlikely]] return acc;
			const auto& b_pos = sph_data->get_position(j);
			double c_a = _avg_speed_of_sound(densities[i], one_over_rd, rp, gamma);
			double c_b = _avg_speed_of_sound(densities[j], one_over_rd, rp, gamma);
			double c_ab = (c_a + c_b); // why not divide by 2??
			double mu_ab = _mu_ab(sph_data->get_velocity(i), sph_data->get_velocity(j), sph_data->get_position(i), b_pos, h, eps);
			double pi_ab = _pi_ab(c_ab, mu_ab, densities[i], densities[j], alpha, beta);
			Vector ab = sph_data->get_position(i) - b_pos;
			double distance = ab.magnitude();
			ab.normalize();
			return acc + sph_data->get_mass(j) * pi_ab * _kernel->gradient(distance, ab);
		}
	);
}

double SPHViscocityForce::_avg_speed_of_sound(double density, double one_over_rest_density, double rest_pressure, double gamma) const noexcept {
	return std::sqrt(gamma * one_over_rest_density * rest_pressure * pow(density * one_over_rest_density, gamma - 1));
}

double SPHViscocityForce::_mu_ab(const Vector& vel_a, const Vector& vel_b, const Vector& pos_a, const Vector& pos_b, double h, double epsilon) const {
	Vector vel_diff = vel_a - vel_b;
	Vector pos_diff = pos_a - pos_b;
	double mag_pos_diff_sq = pos_diff * pos_diff;
	double dot = vel_diff * pos_diff;
	return (dot * h) / (mag_pos_diff_sq + epsilon * h * h);
}

double SPHViscocityForce::_pi_ab(double c_ab, double mu_ab, double density_a, double density_b, double alpha, double beta) const {
	return (-alpha * c_ab * mu_ab + beta * mu_ab * mu_ab) / (density_a + density_b);
}