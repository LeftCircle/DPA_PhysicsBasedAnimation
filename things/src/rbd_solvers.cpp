#include "rbd_solvers.h"

using namespace pba;

Vector pba::rbd_single_particle_pos_rot_update(const RB_sp& rbd, const size_t idx, const double dt){
    Vector rotor = rbd->angular_velocity * dt;
    Matrix angular_rotation = rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->angular_rotation;
    Vector center_of_mass = rbd->center_of_mass + rbd->linear_velocity * dt;
    return angular_rotation * rbd->get_lever_arm(idx) + center_of_mass;
}

void AdvanceRotationAndCOM::solve(const double dt) {
	solve(_rbd, dt);
    auto pos = _rbd->get_vector_attribute_span("positions");
    auto updated_pos = _rbd->get_vector_attribute_span("updated_positions");
    std::copy(std::execution::par, updated_pos.begin(), updated_pos.end(), pos.begin());
}

AdvanceRotationAndCOMWithCollisions::AdvanceRotationAndCOMWithCollisions(
	RB_sp rbd,
	std::shared_ptr<RBDCollisionHandler> rbd_ch
    ) : AdvanceRotationAndCOM(rbd), rbd_coll_handler(rbd_ch)
{
    if (!_rbd->has_vector_attribute("updated_positions")){
        _rbd->add_attribute<Vector>("updated_positions", DSAv());
    }
}

void AdvanceRotationAndCOMWithCollisions::solve(const double dt) {
    AdvanceRotationAndCOM::solve(_rbd, dt);
    rbd_coll_handler->handle_collisions(_rbd, "updated_positions", dt);
	auto pos = _rbd->get_vector_attribute_span("positions");
	auto updated_pos = _rbd->get_vector_attribute_span("updated_positions");
	std::copy(std::execution::par, updated_pos.begin(), updated_pos.end(), pos.begin());
}

void AdvanceRotationAndCOM::solve(RB_sp rbd, const double dt) {
    // determine the current position of all the particles
    auto pos = rbd->get_vector_attribute_span("positions");
    auto updated_pos = rbd->get_vector_attribute_span("updated_positions");
    #pragma omp parallel for
    for (int i = 0; i < rbd->n_particles(); i++){
        pos[i] = rbd->get_vert_pos(i);
    }
    Vector rotor = rbd->angular_velocity * dt;
    rbd->angular_rotation = rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->angular_rotation;
    rbd->compute_moi();
    rbd->center_of_mass += rbd->linear_velocity * dt;
    #pragma omp parallel for
    for (int i = 0; i < rbd->n_particles(); i++){
        updated_pos[i] = rbd->get_vert_pos(i);
    }

}


void AdvanceAngularVelocityAndVelocity::solve(const double dt){
    _torque.compute(_rbd, dt); // updates angular accel and center of mass acceleration
    _rbd->angular_momentum += _rbd->angular_accel * dt;
    _rbd->angular_velocity = _rbd->get_inverse_moi() * _rbd->angular_momentum;
    _rbd->linear_velocity += _rbd->com_accel * dt;
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
    RBDHitResult min_hit{2 * dt, 0, Vector(0, 0, 0), Vector(0, 0, 0), nullptr};
	_handle_rbd_collisions(rbd_sp, min_hit, dt);
}

void RBDCollisionHandler::_handle_rbd_collisions(RB_sp rbd, RBDHitResult& min_hit, const double dt) {
	const size_t n = rbd->n_particles();
	auto updated_positions = rbd->get_vector_attribute_span("updated_positions");
	auto start_positions = rbd->get_vector_attribute_span("positions");
	if (updated_positions.size() != start_positions.size()) [[unlikely]] {
		throw std::runtime_error("updated positions and positions don't match somehow");
	}
	auto m = rbd->get_float_attribute_span("mass");
    
	for (size_t i = 0; i < rbd->n_particles(); i++){
		for (auto& cs : collision_surfaces){
			const std::vector<CollisionObject_sp>& cobjs = cs->get_collision_objects();
			for(size_t j = 0; j < cobjs.size(); j++){
                std::optional<RBDHitResult> hit = bisect_collision(rbd, i, cobjs[j], cs, min_hit.time);
				if (hit && std::abs(hit->time) < std::abs(min_hit.time)){
                    min_hit = hit.value();
                }
			} // end looping through collision objects
		}  // end looping through collision surfaces
	} // end looping through particles
	
	// now we have the earliest time and hit position
	if (min_hit.time < dt){
		// now let's update the com position and rotation to the hit point
		AdvanceRotationAndCOM::solve(rbd, min_hit.time);
		// Now we have to make the rbd bounce
		// conserve kinetic energy
		Vector nxr = min_hit.normal ^ rbd->get_rotated_lever_arm(min_hit.particle);
		double A_numerator = 2.0 * rbd->linear_velocity * min_hit.normal + (m[min_hit.particle] / rbd->get_total_mass()) * (
			rbd->angular_velocity * (nxr));
		double A_denom = 1 + m[min_hit.particle] * m[min_hit.particle] / rbd->get_total_mass() * (
			nxr * rbd->get_inverse_moi() * nxr);
		
		double A = - A_numerator / A_denom;

		// Now update pos and rotation with the bounce
		rbd->linear_velocity += A * min_hit.normal;
		rbd->angular_velocity += A * m[min_hit.particle] * rbd->get_inverse_moi() * nxr;
		
		// Now we have to update position/rotation
		double time_left = dt - min_hit.time;
		AdvanceRotationAndCOM::solve(rbd, time_left);
		// yay recursive function call!
		_handle_rbd_collisions(rbd, min_hit, time_left);
	}
}

std::optional<RBDHitResult> pba::bisect_collision(
	const RB_sp& rbd, size_t particle_idx,
	const CollisionObject_sp& cobj, CollisionSurface_sp cs,
	double max_t)
{
	const Vector& n = cobj->get_normal();
	double f_start = n * rbd->get_position(particle_idx);
	// predict x_end for JUST this one particle
	Vector x_end = rbd_single_particle_pos_rot_update(rbd, particle_idx, max_t);
	double f_end = n * x_end;

	// Early out if no collisions
	if (f_start * f_end > 0){
		return std::nullopt;
	}
	// We have a collision! Time to bisect
	double t0 = 0, t1 = max_t, f1 = f_start, th;
	Vector x_mid;
	for (int step = 0; step <= MAX_BISEC_ITERS; step++){
		th = (t0 + t1) / 2.0;
		// Do we have to swap the direction of the update each time?
		x_mid = rbd_single_particle_pos_rot_update(rbd, particle_idx, th);
		double fmid = x_mid * n;
		if (std::abs(fmid) < BISEC_TOLERANCE){
			Vector norm = (rbd->center_of_mass - x_mid) * n > 0 ? n : -n;
			return RBDHitResult{th, particle_idx, x_mid, norm, cs};
		}
		if (f1 * fmid > 0){
			t0 = th;
		} else {
			t1 = th;
		}
	}
	// did not converge. Just return what we have
	Vector norm = (rbd->center_of_mass - x_mid) * n > 0 ? n : -n;
	return RBDHitResult{th, particle_idx, x_mid, norm, cs};
}

