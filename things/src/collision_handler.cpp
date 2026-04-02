#include "collision_handler.h"


using namespace pba;



void CollisionHandler::handle_collisions(
	DynamicalStateDataBase_sp dsd,
	const std::string& updated_pos_attr_name,
	const double dt)
{
	const size_t n = dsd->n_particles();
	auto updated_positions = dsd->get_vector_attribute_span(updated_pos_attr_name);
	auto start_positions = dsd->get_vector_attribute_span("positions");
	auto velocities = dsd->get_vector_attribute_span("velocities");
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

void CollisionHandler::_handle_particle_collisions(
													Vector& start_pos,
													Vector& updated_pos,
													Vector& velocity,
													const double dt
	) const 
{
	bool keep_checking = true;
	CollisionHandleInfo earliest_hit;
	CollisionHitInfo temp_hit;
	ParticleUpdateInfo pui(start_pos, updated_pos, velocity, dt);
	while (keep_checking && std::abs(pui.remaining_dt) > EPSILON){
		keep_checking = _check_for_collision_against_all_surfaces(earliest_hit, temp_hit, pui);
		if( keep_checking ){
			_on_collision_detected(earliest_hit, pui);
		}
	}
}

bool CollisionHandler::_check_for_collision_against_all_surfaces(
	CollisionHandleInfo& earliest_hit,
	CollisionHitInfo& temp_hit,
	ParticleUpdateInfo& pui) const
{
	bool keep_checking = false;
	earliest_hit.hit_info.time_of_impact = 2.0 * pui.remaining_dt;
	temp_hit.time_of_impact = 2.0 * pui.remaining_dt;
	for( const auto& cs : collision_surfaces ){
		// if there is no hit, cs -> packs 2.0 * pui.remaining_dt into temp_hit
		bool hit = cs->hit(pui.start_pos, pui.updated_pos, pui.velocity, pui.remaining_dt, temp_hit);
		bool earlier_hit = std::abs(temp_hit.time_of_impact) < std::abs(earliest_hit.hit_info.time_of_impact);
		if( hit && earlier_hit){
			earliest_hit.hit_info = temp_hit;
			earliest_hit.collision_surface = cs;
			keep_checking = true;
		}
	}
	return keep_checking;
}

void CollisionHandler::_on_collision_detected(CollisionHandleInfo& earliest_hit, ParticleUpdateInfo& pui) const noexcept{
	pui.remaining_dt = pui.remaining_dt - earliest_hit.hit_info.time_of_impact;
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
	
	double dist_to_plane = (new_position - collision_position) * hit_normal;
	if (dist_to_plane < MIN_END_DIST_FROM_COLLISION) {
		new_position += hit_normal * (MIN_END_DIST_FROM_COLLISION);
	}
	return new_position;

}

void RBDCollisionHandler::handle_collisions(
	DynamicalStateDataBase_sp dsd,
	const std::string& updated_pos_attr_name,
	const double dt
) {
	auto rbd_sp = std::dynamic_pointer_cast<RigidBodyStateData>(dsd);
	// TODO -> We could have general solver/update classes and then just have the different dynamical state data
	// feed everything into uniforms... then we would only ever need the dsd and could use any collision/force. 
	if (!rbd_sp) throw std::runtime_error("RBDCollision handler requires rbd");
	_handle_rbd_collisions(rbd_sp);
}

 void RBDCollisionHandler::register_collision_surface(const CollisionSurface_sp cs){
	CollisionHandler::register_collision_surface(cs);
	_collision_handle_data.emplace(cs, RBD_CollisionSurfaceInfo());
	// Now we need to resize all of the maps to have the correct size
	size_t n_objects = collision_surfaces.size();
	for (auto& csinfo : _collision_handle_data){
		csinfo.second.collision_time.resize(n_objects);
		csinfo.second.plane_implicit_end.resize(n_objects);
		csinfo.second.plane_implicit_start.resize(n_objects);
		csinfo.second.colliding_particle.resize(n_objects);
	}
 }

void RBDCollisionHandler::_handle_rbd_collisions(RB_sp rbd) {
	const size_t n = rbd->n_particles();
	auto updated_positions = rbd->get_vector_attribute_span("updated_positions");
	auto start_positions = rbd->get_vector_attribute_span("positions");
	
	// We need to evaluate the plane implicit function for each collision object
	// kind of silly but I want vectors for the implicit results for all of the
	// collision planes
	for (size_t i = 0; i < rbd->n_particles(); i++){
		for (auto& cs : collision_surfaces){
			auto& pi_start = _collision_handle_data[cs].plane_implicit_start;
			const std::vector<CollisionObject_sp>& cobjs = cs->get_collision_objects();
			std::transform(std::execution::par_unseq, cobjs.begin(), cobjs.end(),
				pi_start.begin(),
				[](const Vector& pos){
					return 0;
				} 
			);
		}
	}
	// For each 
	
}