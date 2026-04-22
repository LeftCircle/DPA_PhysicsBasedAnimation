#include "particle_rbd_collisions.h"

using namespace pba;


void ParticleRBDCollisionHandler::handle_collisions(
	DynamicalStateDataBase_sp dsd,
	const std::string& updated_pos_attr_name,
	const double dt
){
	// This assumes that all particles have updated positions
	// So start by checking for earliest particle collision, then earliest rbd collisions

	// 1.) find earliest particle static geo collision (we might not need this)
	CollisionHandleInfo earliest_hit = _find_earliest_particle_static_geo_collision(
		dsd, "updated_positions", dt
	);

	// 2.) find earliest rbd static geo collision
	RBDHitResult earliest_rbd_hit = _find_earliest_rbd_static_collision_from_all_rbds(dt);

	// 3.) find earliest particle rbd triangle collision
	// Will need a different data structure that tracks what triangle of what rbd is hit
	ParticleRBDHitResult earliest_particle_triangle_hit = _find_earliest_particle_rbd_collision(dsd, dt);
	
	// 4.) advance all to the time of earliest collision

	// 5.) determine new velocities for the colliding particles/rbds based on collision type

	// 6.) repeat until there are no more collisions this timestep. 

}

RBDHitResult ParticleRBDCollisionHandler::_find_earliest_rbd_static_collision_from_all_rbds(const double dt) const{
	RBDHitResult earliest_rbd_hit{2.0 * dt};
	for (auto rbd: _rbds){
		RBDHitResult temp_hit{2.0 * dt};
		bool hit = _find_earliest_rbd_static_collision(rbd, temp_hit, dt);
		if (std::abs(temp_hit.time) < std::abs(earliest_rbd_hit.time)){
			earliest_rbd_hit = temp_hit;
		}
	}
	return earliest_rbd_hit;
}

ParticleRBDHitResult ParticleRBDCollisionHandler::_find_earliest_particle_rbd_collision(DSDB_sp dsd, const double dt) const 
{
	auto start_positions = dsd->get_vector_attribute_span("positions");
	auto updated_positions = dsd->get_vector_attribute_span("updated_positions");
	auto velocities = dsd->get_vector_attribute_span("velocities");
	ParticleRBDHitResult earliest_particle_triangle_hit{2.0 * dt};
	for (size_t i = 0; i < dsd->n_particles(); i++){
		for (auto& rbd: _rbds){
			// rbds can have a vector attribute that contains all of the indices for each face of the mesh
			auto tri_indices = rbd->get_vector_attribute_span("tri_indices");
			for (auto& tri_vec : tri_indices){
				// We can run collision checks against each tri here
				Triangle tri_start(
					rbd->get_rotated_lever_arm(tri_vec.X()),
					rbd->get_rotated_lever_arm(tri_vec.Y()),
					rbd->get_rotated_lever_arm(tri_vec.Z())
				);
				Triangle tri_end(
					rbd_single_particle_pos_rot_update(rbd, tri_vec.X(), dt),
					rbd_single_particle_pos_rot_update(rbd, tri_vec.Y(), dt),
					rbd_single_particle_pos_rot_update(rbd, tri_vec.Z(), dt)
				);
				if (does_moving_particle_collide_with_moving_rbd_tri(
					start_positions[i],
					updated_positions[i],
					tri_start, tri_end
				)){
					// Bisect to find the collision time
					ParticleRBDHitResult result = _bisect_particle_rbd_collision(
						i, start_positions[i], velocities[i], tri_vec, rbd, dt
					);
					if (result.time < earliest_particle_triangle_hit.time){
						earliest_particle_triangle_hit = result;
					}
				}
			}
		}
	}
	return earliest_particle_triangle_hit;
}

bool ParticleRBDCollisionHandler::does_moving_particle_collide_with_moving_rbd_tri(
	Vector p_start,
	Vector p_end,
	Triangle tri_start,
	Triangle tri_end
) const {
	double f0 = (p_start - tri_start.v0) * tri_start.get_normal();
	double f1 = (p_end - tri_end.v0) * tri_end.get_normal();
	return f0 * f1 > 0;
}

ParticleRBDHitResult ParticleRBDCollisionHandler::_bisect_particle_rbd_collision(
	size_t particle_idx,
	Vector p_start,
	Vector p_vel,
	Vector rbd_indices,
	RB_sp rbd,
	double dt
) const{
	Triangle tri_start(
		rbd->get_rotated_lever_arm(rbd_indices.X()),
		rbd->get_rotated_lever_arm(rbd_indices.Y()),
		rbd->get_rotated_lever_arm(rbd_indices.Z())
	);
	const Vector& n = tri_start.get_normal();
	double t0 = 0, t1 = dt, f1, th;
	Vector particle_mid;
	Triangle tri_mid;
	for (int step = 0; step <= MAX_BISEC_ITERS; step++){
		th = (t0 + t1) / 2.0;
		tri_mid.set(
			rbd_single_particle_pos_rot_update(rbd, rbd_indices.X(), dt),
			rbd_single_particle_pos_rot_update(rbd, rbd_indices.Y(), dt),
			rbd_single_particle_pos_rot_update(rbd, rbd_indices.Z(), dt)
		);
		Vector updated_norm = tri_mid.get_normal();
		particle_mid = p_start + p_vel * th;
		double fmid = (particle_mid - tri_mid.v0) * updated_norm;
		if (std::abs(fmid) < BISEC_TOLERANCE && fmid * f1 > 0){
			// The normal for collision surfaces should point to whatever side the particle
			// starts on.
			Vector norm = (p_start - tri_start.v0) * tri_start.get_normal() > 0 ? updated_norm : -updated_norm;
            return ParticleRBDHitResult{th, particle_idx, rbd_indices, particle_mid, norm, true};
		}
		if (f1 * fmid > 0){
			t0 = th;
		} else {
			t1 = th;
		}
	}
	// did not converge. Just return what we have
    printf("Did not converge th = %f\n", th);
	Vector norm = (p_start - tri_start.v0) * tri_start.get_normal() > 0 ? tri_mid.get_normal() : -tri_mid.get_normal();
	return ParticleRBDHitResult{th, particle_idx, rbd_indices, particle_mid, norm, false};
}