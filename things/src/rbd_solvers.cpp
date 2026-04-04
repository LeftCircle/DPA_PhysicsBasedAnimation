#include "rbd_solvers.h"

using namespace pba;

Vector pba::rbd_single_particle_pos_rot_update(const RB_sp& rbd, const size_t idx, const double dt){
    Vector rotor = rbd->angular_velocity * dt;
    double angle = rotor.magnitude();
    Matrix angular_rotation = (angle > 1e-12)
        ? rotation(rotor / angle, -angle) * rbd->angular_rotation
        : rbd->angular_rotation;
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
    //AdvanceRotationAndCOM::solve(_rbd, dt);
    rbd_coll_handler->handle_collisions(_rbd, "updated_positions", dt);
	// auto pos = _rbd->get_vector_attribute_span("positions");
	// auto updated_pos = _rbd->get_vector_attribute_span("updated_positions");
	// std::copy(std::execution::par, updated_pos.begin(), updated_pos.end(), pos.begin());
}

void AdvanceRotationAndCOM::solve(RB_sp rbd, const double dt) {
    Vector rotor = rbd->angular_velocity * dt;
    const Vector& av = rbd->angular_velocity;
    double angle = rotor.magnitude();
    if (angle > 1e-12){
        rbd->angular_rotation = rotation(rotor.unitvector(), -rotor.magnitude()) * rbd->angular_rotation;
    }
    rbd->compute_moi();
    rbd->center_of_mass += rbd->linear_velocity * dt;
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
	if (!rbd_sp) throw std::runtime_error("RBDCollision handler requires rbd");
    RBDHitResult min_hit{2 * dt, 0, Vector(0, 0, 0), Vector(0, 0, 0), nullptr};
	_handle_rbd_collisions(rbd_sp, min_hit, dt);
}

void RBDCollisionHandler::_handle_rbd_collisions(RB_sp rbd, RBDHitResult& min_hit, double dt) {
	const size_t n = rbd->n_particles();
	auto m = rbd->get_float_attribute_span("mass");
    while (std::abs(min_hit.time) > 0.0001){
        bool hit_found = false;
        for (size_t i = 0; i < rbd->n_particles(); i++){
            for (auto& cs : collision_surfaces){
                const std::vector<CollisionObject_sp>& cobjs = cs->get_collision_objects();
                for(size_t j = 0; j < cobjs.size(); j++){
                    std::optional<RBDHitResult> hit = bisect_collision(rbd, i, cobjs[j], cs, dt);
                    if (hit && std::abs(hit->time) < std::abs(min_hit.time)){
                        min_hit = hit.value();
                        hit_found = true;
                    }
                } // end looping through collision objects
            }  // end looping through collision surfaces
        } // end looping through particles
        
        // now we have the earliest time and hit position
        if (hit_found){
            // now let's update the com position and rotation to the hit point
            AdvanceRotationAndCOM::solve(rbd, min_hit.time);
            
            // Now we have to make the rbd bounce
            // conserve kinetic energy
            // I tried to derive this from conservation of energy and cannot
            Vector nxr = min_hit.normal ^ rbd->get_rotated_lever_arm(min_hit.particle);
            double A_numerator = (2.0 * rbd->linear_velocity * min_hit.normal) + ((m[min_hit.particle] / rbd->get_total_mass()) * (rbd->angular_velocity * nxr));
            double A_denom = 1.0 + (m[min_hit.particle] * m[min_hit.particle] / rbd->get_total_mass() * (nxr * rbd->get_inverse_moi() * nxr));
            double A = -A_numerator / A_denom;
            
            // Now update pos and rotation with the bounce
            rbd->linear_velocity += A * min_hit.normal;
            rbd->angular_velocity += A * m[min_hit.particle] * rbd->get_inverse_moi() * nxr;
            
            // now scale by sticky and restitution?
            double ks = min_hit.surface->get_sticky();
            double kr = min_hit.surface->get_restitution();
            const Vector& lv = rbd->linear_velocity;
            const Vector& n = min_hit.normal;
            rbd->linear_velocity = (ks * (lv - (lv * n) * n)) + kr * (lv * n) * n;
            // I have no clue how to update the angular velocity here
            rbd->angular_velocity *= (ks + kr) / 2;

            rbd->angular_momentum = rbd->get_moi() * rbd->angular_velocity;

            // Now we have to update position/rotation
            double time_left = dt - min_hit.time;
            // We don't actually advance yet. Leave that to the bisecting to predict. 
            // Now do it again with min_hit_time
            dt = time_left;
            min_hit.time = time_left;
            // move it up a smidgen if no convergence. Keeps all particles on the same side
            // of the collision plane
            float n_dot_vec_from_plane_to_hit_pos = min_hit.normal * (min_hit.position - min_hit.point_on_cobj);
            if (n_dot_vec_from_plane_to_hit_pos < 0){ //!min_hit.converged){//} &&
                rbd->center_of_mass -= 2 * n_dot_vec_from_plane_to_hit_pos * min_hit.normal;
                //printf("Adjusted COM\n");
            }
            // This is a bit of a hack to avoid an infinite loop and to keep the bunny from freezing
            // when it is on a surface
            if (min_hit.time < 0.00000001){
                //printf("Adjust slightly above plane");
                rbd->center_of_mass += 0.000001 * min_hit.normal;
            }
        } else {
            // no collision! 
            // finish advancing
            AdvanceRotationAndCOM::solve(rbd, dt);
            return;
        }
    }
}

std::optional<RBDHitResult> pba::bisect_collision(
	const RB_sp& rbd, size_t particle_idx,
	const CollisionObject_sp& cobj, CollisionSurface_sp cs,
	double max_t)
{
	const Vector& n = cobj->get_normal();
	double f_start = n * (rbd->get_vert_pos(particle_idx) - cobj->get_point_on_obj());
	// predict x_end for JUST this one particle
	Vector x_end = rbd_single_particle_pos_rot_update(rbd, particle_idx, max_t);
	double f_end = n * (x_end - cobj->get_point_on_obj());

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
		double fmid = (x_mid - cobj->get_point_on_obj()) * n;
		if (std::abs(fmid) < BISEC_TOLERANCE){// && fmid * f1 > 0){
            // We need to check if the collision is actually on the object.
            // like for a triangle if it is within the barycentric coords
            bool is_within = cobj->is_on_surface(x_mid);
            if (!is_within){
                return std::nullopt;
            }
			Vector norm = (rbd->center_of_mass - x_mid) * n > 0 ? n : -n;
            return RBDHitResult{th, particle_idx, x_mid, norm, cs, true, cobj->get_point_on_obj()};
		}
		if (f1 * fmid > 0){
			t0 = th;
		} else {
			t1 = th;
		}
	}
	// did not converge. Just return what we have
    printf("Did not converge\n");
	Vector norm = (rbd->center_of_mass - x_mid) * n > 0 ? n : -n;
	return RBDHitResult{th, particle_idx, x_mid, norm, cs, false, cobj->get_point_on_obj()};
}

