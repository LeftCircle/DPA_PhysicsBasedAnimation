#include "partial_solvers.h"


using namespace pba;

void PartialSolverAdvancePosition::solve(const double dt){
	const size_t n = _state_data->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		const Vector& pos = _state_data->get_position(i);
		const Vector& vel = _state_data->get_velocity(i);
		_state_data->set_position(i, pos + vel * static_cast<float>(dt) );
	}
}

AdvancePositionWithCollisions::AdvancePositionWithCollisions(DynamicalStateData_sp dsd, CollisionHandler_sp collision_handler) :
	_state_data(dsd), _collision_handler(collision_handler) {
	if (!dsd->has_vector_attribute("new_positions")) {
		// Create the new_positions attribute if it doesn't exist
		dsd->add_attribute<Vector>("new_positions", DSAv("new_positions", Vector(0.0f, 0.0f, 0.0f)));
	}
}

void AdvancePositionWithCollisions::solve(const double dt){
	const size_t n = _state_data->n_particles();
	auto new_positions = _state_data->get_vector_attribute_span("new_positions");
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		const Vector& pos = _state_data->get_position(i);
		const Vector& vel = _state_data->get_velocity(i);
		new_positions[i] = pos + vel * dt;
	}
	_collision_handler->handle_collisions(_state_data, "new_positions", dt);
}

void AdvanceVelocityWithForces::solve(const double dt){
	_force_system->compute(_state_data, dt);
	// At this point the accelerations have all been computed. Time to update velocities
	const size_t n = _state_data->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		const Vector& vel = _state_data->get_velocity(i);
		const Vector& acc = _state_data->get_acceleration(i);
		_state_data->set_velocity(i, vel + acc * dt);
	}
}