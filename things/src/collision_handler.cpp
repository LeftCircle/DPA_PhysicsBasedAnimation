#include "collision_handler.h"


using namespace pba;



void CollisionHandler::handle_collisions(DynamicalStateData_sp dsd, const std::string& updated_pos_attr_name, const double dt){
	const size_t n = dsd->n_particles();
	std::span<Vector> updated_positions = dsd->get_vector_attribute_span(updated_pos_attr_name);
	std::span<Vector> start_positions = dsd->get_vector_attribute_span("positions");
	std::span<Vector> velocities = dsd->get_vector_attribute_span("velocities");
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		Vector& start_pos = start_positions[i];
		Vector& updated_pos = updated_positions[i];
		Vector& velocity = velocities[i];
		_handle_particle_collisions(start_pos, updated_pos, velocity, dt);
		// Now set the final position to the updated one
		start_positions[i] = updated_positions[i];
	}
}

void CollisionHandler::_handle_particle_collisions(Vector& start_pos, Vector& updated_pos, Vector& velocity, const double dt) const{
	bool keep_checking = true;
	CollisionHandleInfo earliest_hit;
	CollisionHitInfo temp_hit;
	ParticleUpdateInfo pui(start_pos, updated_pos, velocity, dt);
	int iter = 0;
	while (keep_checking && std::abs(pui.remaining_dt) > EPSILON){
		if (abs(pui.remaining_dt) > abs(dt)){
			printf("Sign changed! original dt: %f, remaining dt: %f\n", dt, pui.remaining_dt);
			break;
		}
		keep_checking = _check_for_collision_against_all_surfaces(earliest_hit, temp_hit, pui);
		if( keep_checking ){
			_on_collision_detected(earliest_hit, pui);
		}
		iter++;
		if (iter > 100000) {
			//std::cerr << "Warning: Collision handling iteration limit reached. This likely means a particle is stuck in a collision loop. Remaining dt: " << pui.remaining_dt << std::endl;
			printf("Original dt = %f\n", dt);
			printf("Remaining dt = %f\n", pui.remaining_dt);
			int test = 0;
			test += 1;
			iter = 0;
			break;
		}
	}
}

bool CollisionHandler::_check_for_collision_against_all_surfaces(CollisionHandleInfo& earliest_hit, CollisionHitInfo& temp_hit, ParticleUpdateInfo& pui) const{
	bool keep_checking = false;
	double no_coll_time = pui.remaining_dt > 0 ? NO_COLLISION : NO_COLLISION_NEG;
	earliest_hit.hit_info.time_of_impact = no_coll_time;
	temp_hit.time_of_impact = no_coll_time;
	for( const auto& cs : collision_surfaces ){
		cs->hit(pui.start_pos, pui.updated_pos, pui.velocity, pui.remaining_dt, temp_hit);
		// now we have to account for negative dt so we get this mess
		bool earlier_hit = std::abs(temp_hit.time_of_impact) < std::abs(earliest_hit.hit_info.time_of_impact);
		if( earlier_hit ){
			earliest_hit.hit_info = temp_hit;
			earliest_hit.collision_surface = cs;
			keep_checking = true;
		}
	}
	return keep_checking;
}

void CollisionHandler::_on_collision_detected(CollisionHandleInfo& earliest_hit, ParticleUpdateInfo& pui) const noexcept{
	// Now we actually have to handle the collision
	pui.remaining_dt = pui.remaining_dt > 0 ? std::max(pui.remaining_dt - earliest_hit.hit_info.time_of_impact, 0.0) : std::min(pui.remaining_dt - earliest_hit.hit_info.time_of_impact, 0.0);
	pui.updated_pos = _resolve_collision_against_static_object(
		pui,
		earliest_hit.hit_info.position,
		earliest_hit.hit_info.normal,
		earliest_hit.collision_surface->get_restitution(),
		earliest_hit.collision_surface->get_sticky()
	);
	// Now set the start position to the collision position for the next iteration
	// plus a very small epsilon to prevent rehitting the same or similar surfaces
	pui.start_pos = earliest_hit.hit_info.position + earliest_hit.hit_info.normal * MIN_END_DIST_FROM_COLLISION;
}

Vector CollisionHandler::_resolve_collision_against_static_object(
	ParticleUpdateInfo& pui,
	const Vector& collision_position,
	const Vector& hit_normal,
	const double restitution, 
	const double sticky
) const noexcept {
	
	// Determine the new velocity after collision
	pui.velocity = sticky * pui.velocity  - (sticky + restitution) * (pui.velocity * hit_normal) * hit_normal;
	Vector new_position = collision_position + pui.velocity * pui.remaining_dt;
	// We have to ensure that the end position is far enough away from the collision plane
	// This prevents multi collisions and also keeps the particle on the correct side of the collision object
	// as long as the particle ends on the same side that it started!
	double dist_to_plane = (new_position - collision_position) * hit_normal;
	if (dist_to_plane < MIN_END_DIST_FROM_COLLISION) {
		// figure out dir from end pos to start pos
		//Vector end_to_start_dir = (pui.start_pos - new_position).unitvector();
		double dir_plane_to_start_dot = (pui.start_pos - collision_position) * hit_normal;
		if (dir_plane_to_start_dot > 0) {
			// The start position is on the same side of the plane as the end position,
			new_position += hit_normal * (MIN_END_DIST_FROM_COLLISION + EPSILON);
		} else {
			new_position -= hit_normal * (MIN_END_DIST_FROM_COLLISION + EPSILON);
		}
	}
	return new_position;

}