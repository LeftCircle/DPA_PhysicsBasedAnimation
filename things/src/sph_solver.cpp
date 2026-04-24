#include "sph_solver.h"

using namespace pba;


double pba::get_density_with_uniform_h_parallel(
    size_t i,
    const span<const size_t>& neighbor_indices,
    const DynamicalStateData_sp& dsd,
    const Kernel& kernel)
{
    const Vector& pos_i = dsd->get_position(i);
	const auto positions = dsd->get_vector_attribute_span("positions");
	const auto masses = dsd->get_float_attribute_span("mass");
	return std::transform_reduce(
		std::execution::par_unseq,
		neighbor_indices.begin(),
		neighbor_indices.end(),
		0.0,
		std::plus<>(),
		[&pos_i, &positions, &masses, &kernel](size_t j){
			double distance = (pos_i - positions[j]).magnitude();
			return masses[j] * kernel(distance);
		}
	);
}

double pba::get_density_with_uniform_h(
    size_t i,
    const span<const size_t>& neighbor_indices,
    const DynamicalStateData_sp& dsd,
    const Kernel& kernel)
{
	const Vector& pos_i = dsd->get_position(i);
	const auto positions = dsd->get_vector_attribute_span("positions");
	const auto masses = dsd->get_float_attribute_span("mass");
	return std::accumulate(neighbor_indices.begin(), neighbor_indices.end(), 0.0, [&pos_i, &positions, &masses, &kernel](double acc, size_t j){
        double distance = (pos_i - positions[j]).magnitude();
        return acc + masses[j] * kernel(distance);
    });
}

double pba::get_density_with_uniform_h_silly_loop(
    size_t i,
    const span<const size_t>& neighbor_indices,
    const DynamicalStateData_sp& dsd,
    const Kernel& kernel)
{
	const Vector& pos_i = dsd->get_position(i);
	double density = 0.0;
	for (size_t j : neighbor_indices) {
		const Vector& pos_j = dsd->get_position(j);
		double mass_j = dsd->get_mass(j);
		double distance = (pos_i - pos_j).magnitude();
		density += mass_j * kernel(distance);
	}
	return density;
}

SPHPositionSolver::SPHPositionSolver(
	SPHData_sp dsd,
	CollisionHandler_sp collision_handler,
	idx_volume_sp occupancy_volume,
	Kernel_sp kernel
) : 
	AdvancePositionWithCollisions(dsd, collision_handler),
	_occupancy_volume(occupancy_volume),
	_kernel(kernel) {
		if (!dsd->has_double_attribute("density")) {
			// Create the density attribute if it doesn't exist
			dsd->add_attribute<double>("density", DSAd("density", 1.0));
		}
}

void SPHPositionSolver::solve(const double dt) {
	AdvancePositionWithCollisions::solve(dt);
	// Now update densities and the occupancy grid
	const auto positions = _state_data->get_vector_attribute_span("positions");
	const auto next_pos = _state_data->get_vector_attribute_span("new_positions");
	_occupancy_volume->populate(
		next_pos,
		[](std::vector<size_t>& cell, size_t idx){
			cell.push_back(idx);
		}
	);

	span<double> densities = _state_data->get_double_attribute_span("density");
	#pragma omp parallel for
	for (size_t i = 0; i < _state_data->n_particles(); i++) {
		double density = _occupancy_volume->accumulate_neighbor_cells(
			i,
			_state_data->get_position(i),
			0.0,
			[&densities, this](size_t idx, double acc, const std::vector<size_t>& neighbor_indices){
				return acc + get_density_with_uniform_h(idx, span<const size_t>(neighbor_indices), _state_data, *_kernel);	
			}
		);
		densities[i] = density;
	}

	std::copy(next_pos.begin(), next_pos.end(), positions.begin());
}

PciSPHPositionSolver::PciSPHPositionSolver(
	SPHData_sp dsd,
	CollisionHandler_sp collision_handler,
	idx_volume_sp occupancy_volume,
	Kernel_sp kernel) : SPHPositionSolver(dsd, collision_handler, occupancy_volume, kernel)
{}

void PciSPHPositionSolver::solve(const double dt) {
	

}

void PciSPHPositionSolver::_accumulate_pressure_force(double dt){
	auto dsd = std::dynamic_pointer_cast<SPHData>(_state_data);
	const size_t n = _state_data->n_particles();
	double target_dens = dsd->rest_density();

	auto m = dsd->get_float_attribute_span("mass");
	auto p = dsd->get_vector_attribute_span("positions");
	auto v = dsd->get_vector_attribute_span("velocities");
	auto new_v = dsd->get_vector_attribute_span("new_velocities");
	auto new_p = dsd->get_vector_attribute_span("new_positions");
	auto p_forces = dsd->get_vector_attribute_span("pressure_force");
	auto pressure_d = dsd->get_double_attribute_span("pressures");
	auto a = dsd->get_vector_attribute_span("acceleration");
	auto ds = dsd->get_double_attribute_span("predicted_density");
	auto dens_error = dsd->get_double_attribute_span("density_error");

	const double delta = _accumulate_delta(dt);
	auto kernel = _kernel;
	
	// init buffers;
	#pragma omp parallel for
	for (size_t i = 0; i < n; i++){
		p_forces[i] = Vector(0, 0, 0);
		pressure_d[i] = 0.0;
	}

	for (unsigned int k = 0; k < _max_iterations; k++){
		// predict vel and position
		#pragma omp parallel for
		for (size_t i = 0; i < n; i++){
			new_v[i] = v[i] + dt * (a[i] + p_forces[i] / m[i]);
			new_p[i] = p[i] + dt * new_v[i]; 
		}
		// resolve collision
		_collision_handler->handle_collisions(dsd, "new_positions", "new_velocities", dt);
		// compute pressure from density error
		#pragma omp parallel for
		for (size_t i = 0; i < n; i++){
			double weight_sum = _occupancy_volume->accumulate_neighbor_cells(
				i,
				new_p[i],
				0.0,
				[&new_p, kernel](size_t index, double weight_sum, const std::vector<size_t>& n_idxs){
					for (size_t j : n_idxs){
						double dist = (new_p[index] - new_p[j]).magnitude();
						weight_sum += (*kernel)(dist);
					}  
					return weight_sum;
				}
			);
			double dens = m[i] * weight_sum;
			double dens_error_i = dens - target_dens;
			double pressure = delta * dens_error_i;

			pressure_d[i] += pressure;
			ds[i] = dens;
			dens_error[i] = dens_error_i;
		}
		
		// Now have to accumulate pressure gradient forces??
		_accumulate_pressure_gradient_force(p, ds, pressure_d, p_forces);
		// compute max density error
		double max_density_error = *std::max_element(dens_error.begin(), dens_error.end(),
			 [](double a, double b){ return std::max(std::abs(a), std::abs(b)); }
		);

		double density_error_ratio = max_density_error / target_dens;
		if (std::abs(density_error_ratio) < _max_density_error_ratio){
			break;
		}
	}

	// Accumulate pressure force
	#pragma omp parallel for
	for (size_t i = 0; i < n; i++){
		a[i] += p_forces[i] / dsd->get_mass(i);
	}
}

void PciSPHPositionSolver::_accumulate_pressure_gradient_force(
	span<Vector>& pos,
	span<double>& densities,
	span<double>& pressures,
	span<Vector>& pressure_forces)
{
	auto dsd = std::dynamic_pointer_cast<SPHData>(_state_data);
	const double msq = dsd->get_mass(0) * dsd->get_mass(0);
	#pragma omp parallel for
	for (size_t i = 0; i < dsd->n_particles(); i++){
		Vector p_force = _occupancy_volume->accumulate_neighbor_cells(
			i, dsd->get_position(i), Vector(0, 0, 0), 
			[&dsd, &pressure_forces, msq, &pressures, &densities, this](size_t index, Vector p_sum, const std::vector<size_t>& n_idxs){
				for (size_t j : n_idxs){
					Vector vec = dsd->get_position(index) - dsd->get_position(j);
					double dist = vec.magnitude();
					if (dist > 0.0){
						Vector dir = vec / dist;
						p_sum -= ((msq *
							(pressures[index] / (densities[index] * densities[j])) + 
							(pressures[j] / (densities[index] + densities[j]))) *
							_kernel->gradient(dist, dir)
						);
					}
				}
				return p_sum;
			}
		);
		pressure_forces[i] += p_force;
	}
}

double PciSPHPositionSolver::_accumulate_delta(const double dt){
	auto dsd = std::dynamic_pointer_cast<SPHData>(_state_data);
	const double h = dsd->h();
	std::vector<Vector> points(9);
	// get the 9 points. 8 for edges of bounding box, and one in center
	points[0] = Vector(0, 0, 0);
	points[1] = Vector(h, h, h);
	points[2] = Vector(h, h, -h);
	points[3] = Vector(h, -h, h);
	points[4] = Vector(-h, h, h);
	points[5] = Vector(h, -h, -h);
	points[6] = Vector(-h, h, -h);
	points[7] = Vector(-h, -h, h);
	points[8] = Vector(-h, -h, -h);

	double denom = 0;
	Vector denom1(0, 0, 0);
	double denom2 = 0;
	for (size_t i = 0; i < 9; i++){
		const Vector& point = points[i];
		double dsq = point * point;
		if (dsq < h * h){
			double d = std::sqrt(dsq);
			Vector direction = (d > 0.0) ? point / d : Vector(0, 0, 0);
			Vector gradWij = _kernel->gradient(d, direction);
			denom1 += gradWij;
			denom2 += gradWij * gradWij;
		}
	}
	denom += -denom1 * denom1 - denom2;
	return (std::abs(denom) > 0) ? -1.0 / (_compute_beta(dt) * denom) : 0;
}

double PciSPHPositionSolver::_compute_beta(const double dt) {
	auto dsd = std::dynamic_pointer_cast<SPHData>(_state_data);
	double dtm_over_p = dt * dsd->get_mass(0) / dsd->rest_density();
	return 2.0 * dtm_over_p * dtm_over_p;
}

void SPHAdvanceVelocityWithForces::solve(const double dt) {
	_force_system->compute(_state_data, dt);
	// Cast the dsd to SPHData
	SPHData_sp sph_data = std::dynamic_pointer_cast<SPHData>(_state_data);
	// At this point the accelerations have all been computed. Time to update velocities
	const size_t n = sph_data->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		const Vector& vel = sph_data->get_velocity(i);
		const Vector& acc = sph_data->get_acceleration(i);
		Vector clamped_acc = _clamp_vector(acc, sph_data->get_max_particle_acceleration());
		Vector new_vel = vel + clamped_acc * dt;
		new_vel = _clamp_vector(new_vel, sph_data->get_max_particle_speed());
		sph_data->set_velocity(i, new_vel);
	}
}

Vector SPHAdvanceVelocityWithForces::_clamp_vector(const Vector& vec, double max_magnitude) const noexcept {
	double mag = vec.magnitude();
	if (std::isnan(mag)) [[unlikely]] {
		printf("NAAN vector");
		throw std::runtime_error("naan mag");
	}
	if (mag > max_magnitude) {
		Vector clamped_vec = vec;
		clamped_vec.normalize();
		clamped_vec = clamped_vec * max_magnitude;
		return clamped_vec;
	}
	return vec;
}