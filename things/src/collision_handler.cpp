#include "collision_handler.h"
#include "rbd_solvers.h"

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
	_handle_rbd_collisions(rbd_sp, dt);
}

 void RBDCollisionHandler::register_collision_surface(const CollisionSurface_sp cs){
	CollisionHandler::register_collision_surface(cs);
	// TODO -> this does not support adding faces to collision surfaces during sim
	RBD_CollisionSurfaceInfo info;
	size_t n_obj = cs->get_n_collision_objs();
	info.colliding_particle.resize(n_obj);
	info.collision_time.resize(n_obj);
	info.hit_normal.resize(n_obj);
	info.hit_pos.resize(n_obj);
	info.plane_implicit_end.resize(n_obj);
	info.plane_implicit_start.resize(n_obj);
	_collision_handle_data.emplace(cs, info);
 }

void RBDCollisionHandler::_handle_rbd_collisions(RB_sp rbd, const double dt) {
	const size_t n = rbd->n_particles();
	auto updated_positions = rbd->get_vector_attribute_span("updated_positions");
	auto start_positions = rbd->get_vector_attribute_span("positions");
	if (updated_positions.size() != start_positions.size()) [[unlikely]] {
		throw std::runtime_error("updated positions and positions don't match somehow");
	}
	auto m = rbd->get_float_attribute_span("mass");

	// have to reset the collision times
	for (auto& cs : collision_surfaces){
		auto& pi_time = _collision_handle_data[cs].collision_time;
		std::fill(pi_time.begin(), pi_time.end(), 2 * dt);
	}
	
	for (size_t i = 0; i < rbd->n_particles(); i++){
		for (auto& cs : collision_surfaces){
			auto& pi_time = _collision_handle_data[cs].collision_time;
			auto& pi_hp = _collision_handle_data[cs].hit_pos;
			auto& pi_i = _collision_handle_data[cs].colliding_particle;
			auto& pi_norm = _collision_handle_data[cs].hit_normal;

			auto& pi_start = _collision_handle_data[cs].plane_implicit_start;
			auto& pi_end = _collision_handle_data[cs].plane_implicit_end;
			const std::vector<CollisionObject_sp>& cobjs = cs->get_collision_objects();
			for(size_t j = 0; j < cobjs.size(); j++){
				pi_start[j] = (cobjs[j]->get_normal() * start_positions[i]);
				pi_end[j] = (cobjs[j]->get_normal() * updated_positions[i]);
				double f1 = pi_start[j];
				double f2 = pi_end[j];
				if (pi_start[j] * pi_end[j] < 0){
					// Now find the collision
					bool checking = true;
					int n_steps = 0;
					double t0 = 0;
					double t1 = pi_time[j];
					double th, fmid;
					Vector x_mid;
					while (checking){
						th = (t0 + t1) / 2.0;
						Vector rotor = rbd->angular_velocity * th;
    					Matrix ang_rot = rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->angular_rotation;

						x_mid = rbd->center_of_mass + rbd->linear_velocity * th + ang_rot * rbd->get_lever_arm(i);
						fmid = cobjs[j]->get_normal() * x_mid;
						if (std::abs(fmid) < RBD_COLL_TOLERANCE || n_steps > RBD_COLL_MAX_ITERS){
							if (std::abs(th) < std::abs(pi_time[j])){
								// earlier particle collision found for this collision object!
								// TODO -> actually check if a hit occurs at this position 
								// based on collision object type (like for triangles)
								pi_time[j] = th;
								pi_i[j] = i;
								pi_hp[j] = x_mid;
								// set the normal to be in the direction of the center of mass
								Vector norm = cobjs[j]->get_normal();
								Vector hit_to_com = rbd->center_of_mass - x_mid;
								norm = hit_to_com * norm > 0 ? norm : -norm;
								pi_norm[j] = norm;
							}
							checking = false;
						} else {
							if (f1 * fmid > 0){
								f1 = fmid;
								t0 = th;
							} else {
								f2 = fmid;
								t1 = th;
							}
							n_steps++;
						}
					} // end while loop
				} // end collision found for cobj[j]
			} // end looping through collision objects
		}  // end looping through collision surfaces
	} // end looping through particles
	
	// Now we have to find the earliest hit from all the collision objects. kinda sucks with the current data structure
	double earliest_t = 2 * dt;
	Vector earliest_h;
	size_t earliest_p = -1;
	Vector n_eh; // normal earliest hit
	for (auto& cs : collision_surfaces){
		auto& pi_time = _collision_handle_data[cs].collision_time;
		auto& pi_hp = _collision_handle_data[cs].hit_pos;
		auto& hit_p = _collision_handle_data[cs].colliding_particle;
		auto& hit_n = _collision_handle_data[cs].hit_normal;
		for (int i = 0; i < pi_time.size(); i++){
			if (pi_time[i] < earliest_t){
				earliest_t = pi_time[i];
				earliest_h = pi_hp[i];
				earliest_p = hit_p[i];
				n_eh = hit_n[i].unitvector();
			}
		}
	}

	// now we have the earliest time and hit position
	if (earliest_p != -1){
		// now let's update the com position and rotation to the hit point
		AdvanceRotationAndCOM::solve(rbd, earliest_t);
		// Now we have to make the rbd bounce
		// conserve kinetic energy
		// TODO -> are we supposed to use rotated lever arm here???
		Vector nxr = n_eh ^ rbd->get_rotated_lever_arm(earliest_p);
		double A_numerator = 2.0 * rbd->linear_velocity * n_eh + (m[earliest_p] / rbd->get_total_mass()) * (
			rbd->angular_velocity * (nxr));
		double A_denom = 1 + m[earliest_p] * m[earliest_p] / rbd->get_total_mass() * (
			nxr * rbd->get_inverse_moi() * nxr);
		
		double A = - A_numerator / A_denom;

		// Now update pos and rotation with the bounce
		rbd->linear_velocity += A * n_eh;
		rbd->angular_velocity += A * m[earliest_p] * rbd->get_inverse_moi() * nxr;
		
		// Now we have to update position/rotation
		double time_left = dt - earliest_t;
		AdvanceRotationAndCOM::solve(rbd, time_left);
		// yay recursive function call!
		_handle_rbd_collisions(rbd, time_left);
	}
}