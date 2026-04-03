#include "rbd_solvers.h"

using namespace pba;

void AdvanceRotationAndCOM::solve(const double dt) {
	solve(_rbd, dt);
}

AdvanceRotationAndCOMWithCollisions::AdvanceRotationAndCOMWithCollisions(
	RB_sp rbd,
	std::shared_ptr<RBDCollisionHandler> rbd_ch) :
    AdvanceRotationAndCOM(rbd), rbd_coll_handler(rbd_ch) {
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







