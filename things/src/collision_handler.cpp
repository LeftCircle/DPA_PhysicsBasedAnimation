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
		bool keep_checking = true;
		double remaining_dt = dt;
		CollisionHandleInfo earliest_hit;
		CollisionHitInfo temp_hit;
		earliest_hit.hit_info.time_of_impact = NO_COLLISION;
		while (keep_checking && remaining_dt > EPSILON){
			keep_checking = false;
			earliest_hit.hit_info.time_of_impact = NO_COLLISION;
			temp_hit.time_of_impact = NO_COLLISION;
			for( const auto& cs : collision_surfaces ){
				cs->hit(start_pos, updated_pos, velocity, remaining_dt, temp_hit);
				if( temp_hit.time_of_impact < earliest_hit.hit_info.time_of_impact ){
					earliest_hit.hit_info = temp_hit;
					earliest_hit.collision_surface = cs;
					keep_checking = true;
				}
			}
			if( keep_checking ){
				// Now we actually have to handle the collision
				remaining_dt -= earliest_hit.hit_info.time_of_impact;
				updated_pos = _resolve_collision_against_static_object(
					earliest_hit.hit_info.position,
					earliest_hit.hit_info.normal,
					velocity,
					earliest_hit.collision_surface->get_restitution(),
					earliest_hit.collision_surface->get_sticky(),
					remaining_dt
				);

				// Now set the start position to the collision position for the next iteration
				// plus a very small epsilon to prevent rehitting the same or similar surfaces
				start_pos = earliest_hit.hit_info.position + earliest_hit.hit_info.normal * MIN_END_DIST_FROM_COLLISION;
			}
		}
		// Now set the final position to the updated one
		start_positions[i] = updated_positions[i];
	}
}

void CollisionHandler::_handle_particle_collisions(Vector& start_pos, Vector& updated_pos, Vector& velocity, const double dt){

}

Vector CollisionHandler::_resolve_collision_against_static_object(
	const Vector& collision_position,
	const Vector& hit_normal,
	Vector& velocity,
	const double restitution, 
	const double sticky,
	const double dt
){
	// Determine the new velocity after collision
	velocity = sticky * velocity  - (sticky + restitution) * (velocity * hit_normal) * hit_normal;
	Vector new_position = collision_position + velocity * dt;
	// We have to ensure that the end position is far enough away from the collision plane
	// This prevents multi collisions and also keeps the particle on the correct side of the collision object
	double dist_to_plane = (new_position - collision_position) * hit_normal;
	if (dist_to_plane < MIN_END_DIST_FROM_COLLISION) {
		new_position += hit_normal * (MIN_END_DIST_FROM_COLLISION - dist_to_plane);
	}
	return new_position;

}