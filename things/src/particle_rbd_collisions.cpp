#include "particle_rbd_collisions.h"

using namespace pba;


void ParticleRBDCollisionHandler::handle_collisions(
	DSD_sp dsd,
	const std::string& updated_pos_attr_name,
	const double dt
){
	bool collision_found = true;
	double time_left = dt;
	while (collision_found){

		// 1.) find earliest particle static geo collision (we might not need this)
		CollisionHandleInfo earliest_particle_hit = _find_earliest_particle_static_geo_collision(
			dsd, "updated_positions", time_left
		);

		// 2.) find earliest rbd static geo collision
		std::pair<RB_sp, RBDHitResult> earliest_rbd_hit = _find_earliest_rbd_static_collision_from_all_rbds(time_left);

		// 3.) find earliest particle rbd triangle collision
		// Will need a different data structure that tracks what triangle of what rbd is hit
		ParticleRBDHitResult earliest_particle_triangle_hit = _find_earliest_particle_rbd_collision(dsd, time_left);
		
		// 4.) advance all to the time of earliest collision
		double p_t = std::abs(earliest_particle_hit.hit_info.time_of_impact);
		double rbd_t = std::abs(earliest_rbd_hit.second.time);
		double p_rbd_t = std::abs(earliest_particle_triangle_hit.time);
		double abs_2dt = std::abs(2.0 * time_left);
		if (p_t < abs_2dt && p_t < rbd_t && p_t < p_rbd_t){
			// Min time is for particles. Advance particles with collisions and the rest without
			_update_rbd_and_particles_by(dsd, earliest_particle_hit.hit_info.time_of_impact);
			_resolve_particle_collision(dsd, earliest_particle_hit, time_left);
			time_left = time_left - earliest_particle_hit.hit_info.time_of_impact;
		} else if (rbd_t < abs_2dt && rbd_t < p_t && rbd_t < p_rbd_t){
			// Min time is for rbd. Advance rbd with collisions and rest without
			_update_rbd_and_particles_by(dsd, earliest_rbd_hit.second.time);
			_resolve_collision(earliest_rbd_hit.first, earliest_rbd_hit.second, earliest_rbd_hit.second.time);
			time_left = time_left - earliest_rbd_hit.second.time;
		} else if (p_rbd_t < abs_2dt && p_rbd_t < p_t && p_rbd_t < rbd_t){
			// particle rbd collision. Advance particles and rbd, then rest without.
			_update_rbd_and_particles_by(dsd, earliest_particle_triangle_hit.time);
			_resolve_particle_rbd_collision(dsd, earliest_particle_triangle_hit, time_left);
			time_left = time_left - earliest_particle_triangle_hit.time;
		} else{
			collision_found = false;
			// No collisions! update everything by the remaining dt
			_update_rbd_and_particles_by(dsd, time_left);
			break;
		}
	}
}

void ParticleRBDCollisionHandler::_resolve_particle_rbd_collision(DSD_sp dsd, ParticleRBDHitResult& hit_info, const double dt){
	// Everything is already updated to the correct space in time. Now we just need the particle and
	// rbd collision response. 
    const Vector& n = hit_info.normal;
    const Vector& r = hit_info.rbd->get_rotated_lever_arm(hit_info.position);
	const float mp = dsd->get_mass(hit_info.particle);
    const float M = hit_info.rbd->get_total_mass() + mp;
    Vector rxn = r ^ n;
    RB_sp rbd = hit_info.rbd;
	const float rbd_m = rbd->get_total_mass();
	//Vector COM = (mp * dsd->get_position(hit_info.particle) + rbd_m * rbd->center_of_mass) / M;

    // Now we have to make the rbd bounce
    double A_numer_a = (rbd->linear_velocity * M / rbd_m + dsd->get_velocity(hit_info.particle)) * n;
    double A_numer_b = rbd->angular_velocity * rxn;
	double A_denom = 1.0 / mp + M / (rbd_m * rbd_m) + rxn * rbd->get_inverse_moi() * rxn;
    double A = std::abs(2.0 * (A_numer_a + A_numer_b) / A_denom);
    //printf("A = %f\n", A);

    // Now update pos and rotation with the bounce
    rbd->linear_velocity -= A / rbd->get_total_mass() * n;
    rbd->angular_velocity -= A * rbd->get_inverse_moi() * rxn;
	dsd->set_velocity(hit_info.particle, dsd->get_velocity(hit_info.particle) + A / mp * n);
	// Now we have to set the expected update position for this particle
	Vector new_p = dsd->get_position(hit_info.particle) + dsd->get_velocity(hit_info.particle) * (dt - hit_info.time);
	dsd->set_updated_position(hit_info.particle, new_p);

    // Angular momentum update!!!
    rbd->angular_momentum = rbd->get_moi() * rbd->angular_velocity;
}

void ParticleRBDCollisionHandler::_update_rbd_and_particles_by(DSD_sp dsd,const double dt){
	for (auto& rbd: _rbds){	
		AdvanceRotationAndCOM::solve(rbd, dt);
	}
	PartialSolverAdvancePosition::partial_update(dsd, dt);
}

std::pair<RB_sp, RBDHitResult> ParticleRBDCollisionHandler::_find_earliest_rbd_static_collision_from_all_rbds(const double dt) const{
	RBDHitResult earliest_rbd_hit{2.0 * dt};
	std::pair<RB_sp, RBDHitResult> result;
	for (auto rbd: _rbds){
		RBDHitResult temp_hit{2.0 * dt};
		bool hit = _find_earliest_rbd_static_collision(rbd, temp_hit, dt);
		if (std::abs(temp_hit.time) < std::abs(earliest_rbd_hit.time)){
			earliest_rbd_hit = temp_hit;
			result.first = rbd;
		}
	}
	result.second = earliest_rbd_hit;
	return result;
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
			auto tri_indices = rbd->get_face_indices();
			for (auto& cato_tri_vec : tri_indices){
				Vector tri_vec = Vector(cato_tri_vec.x(), cato_tri_vec.y(), cato_tri_vec.z());
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
						earliest_particle_triangle_hit.set_rbd(rbd);
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