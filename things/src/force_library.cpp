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
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		Vector pressure_force = _occupancy_volume->accumulate_neighbor_cells(
			i,
			sph_data->get_position(i),
			Vector(0.0, 0.0, 0.0),
			[&sph_data, &densities, this](size_t idx, Vector acc, const std::vector<size_t>& neighbor_indices){
				const auto neighbor_indices_span(neighbor_indices);
				return acc + _get_pressure_with_uniform_h(
					idx,
					neighbor_indices_span,
					sph_data,
					densities);
			}
		);
		sph_data->set_acceleration(i, sph_data->get_acceleration(i) + pressure_force / sph_data->get_mass(i));

	}
}

Vector SPHPressureForce::_get_pressure_with_uniform_h(
		size_t i,
		const span<const size_t>& neighbor_indices,
		const SPHData_sp sph_data,
		const span<const double>& densities
	) const
{
	const Vector& pos_i = sph_data->get_position(i);
	const auto dens_i = densities[i];
	const auto p_at_i = _get_pressure_at_density(dens_i, sph_data->rest_pressure(), sph_data->rest_density(), sph_data->gamma());
	const auto d_i_sq = dens_i * dens_i;
	Vector pressure_force(0.0, 0.0, 0.0);
	for (size_t j : neighbor_indices) {
		const Vector& pos_j = sph_data->get_position(j);
		double distance = (pos_i - pos_j).magnitude();
		Vector grad_ab = _kernel->gradient(distance, pos_i - pos_j);
		const auto dens_j = densities[j];
		const auto p_at_j = _get_pressure_at_density(dens_j, sph_data->rest_pressure(), sph_data->rest_density(), sph_data->gamma());
		const auto d_j_sq = dens_j * dens_j;
		pressure_force += sph_data->get_mass(j) * (p_at_i / d_i_sq + p_at_j / d_j_sq) * grad_ab;
	}
	return pressure_force;
}

double SPHPressureForce::_get_pressure_at_density(double density, double rest_pressure, double rest_density, double gamma) const {
	return rest_pressure * (pow(density / rest_density, gamma) - 1.0);
}

void SPHViscosityForce::compute_sph(SPHData_sp sph_data, const double dt) const {
	const size_t n = sph_data->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		Vector viscosity_force = _occupancy_volume->accumulate_neighbor_cells(
			i,
			sph_data->get_position(i),
			Vector(0.0, 0.0, 0.0),
			[&sph_data, this](size_t idx, Vector acc, const std::vector<size_t>& neighbor_indices){
				return acc + _compute_viscosity_force_for_neighbors(idx, neighbor_indices, sph_data);
			}
		);
		sph_data->set_acceleration(i, sph_data->get_acceleration(i) + viscosity_force / sph_data->get_mass(i));
	}
}

Vector SPHViscosityForce::_compute_viscosity_force_for_neighbors(size_t i, const span<const size_t>& neighbor_indices, const SPHData_sp sph_data) const {
	const auto densities = sph_data->get_double_attribute_span("density");
	const auto one_over_rd = 1.0 / sph_data->rest_density();
	return -std::accumulate(neighbor_indices.begin(), neighbor_indices.end(), Vector(0.0, 0.0, 0.0), 
		[&sph_data, i, &densities, one_over_rd, this](Vector acc, size_t j){
			const auto& b_pos = sph_data->get_position(j);
			double c_a = _avg_speed_of_sound(densities[i], one_over_rd, sph_data->rest_pressure(), sph_data->gamma());
			double c_b = _avg_speed_of_sound(densities[j], one_over_rd, sph_data->rest_pressure(), sph_data->gamma());
			double c_ab = (c_a + c_b); // why not divide by 2??
			double mu_ab = _mu_ab(sph_data->get_velocity(i), sph_data->get_velocity(j), sph_data->get_position(i), b_pos, sph_data->h(), sph_data->viscosity_epsilon());
			double pi_ab = _pi_ab(c_ab, mu_ab, densities[i], densities[j], sph_data->viscosity_alpha(), sph_data->viscosity_beta());
			Vector ab = sph_data->get_position(i) - b_pos;
			double distance = ab.magnitude();
			if (ab.magnitude() > 0.0)  ab.normalize();
			Vector thing_to_acc = sph_data->get_mass(j) * pi_ab * _kernel->gradient(distance, ab);
			Vector kernel_grad = _kernel->gradient(distance, ab);
			return acc + sph_data->get_mass(j) * pi_ab * _kernel->gradient(distance, ab);
		}
	);
}

double SPHViscosityForce::_avg_speed_of_sound(double density, double one_over_rest_density, double rest_pressure, double gamma) const noexcept {
	return std::sqrt(gamma * one_over_rest_density * rest_pressure * pow(density * one_over_rest_density, gamma - 1));
}

double SPHViscosityForce::_mu_ab(const Vector& vel_a, const Vector& vel_b, const Vector& pos_a, const Vector& pos_b, double h, double epsilon) const {
	Vector vel_diff = vel_a - vel_b;
	Vector pos_diff = pos_a - pos_b;
	double mag_pos_diff_sq = pos_diff * pos_diff;
	double dot = vel_diff * pos_diff;
	return (dot * h) / (mag_pos_diff_sq + epsilon * h * h);
}

double SPHViscosityForce::_pi_ab(double c_ab, double mu_ab, double density_a, double density_b, double alpha, double beta) const {
	return (-alpha * c_ab * mu_ab + beta * mu_ab * mu_ab) / (density_a + density_b);
}

void UniformStrutForce::_compute(std::shared_ptr<SoftBody> sb, const double dt) const {
	// For each edge, determine spring force and friction based on position, rest length, and velocity
	span<const Vector> positions = sb->get_vector_attribute_span("positions");
	span<const Vector> vels = sb->get_vector_attribute_span("velocities");
	const size_t n_edges = sb->edges.size();
	#pragma omp parallel for
	for (size_t i = 0; i < n_edges; i++){
		sb->edges[i].compute(positions, vels, _spring_force, _friction);
	}
	// Now we have to write the forces in. Can't be parallel b/c edges point to multiple points
	for (auto& edge : sb->edges){
		std::pair<size_t, size_t> idxs = edge.get_indices();
		sb->set_acceleration(idxs.first, sb->get_acceleration(idxs.first) + edge.get_force_on_a() / sb->get_mass(idxs.first));
		sb->set_acceleration(idxs.second, sb->get_acceleration(idxs.second) - edge.get_force_on_a() / sb->get_mass(idxs.second));
	}
}