#include "rbd_solvers.h"

using namespace pba;

void AdvanceRotationAndCOM::solve(const double dt) {
    Vector rotor = _rbd->angular_velocity * dt;
    _rbd->angular_rotation = rotation(rotor.unitvector(), -rotor.magnitude()) * _rbd->angular_rotation;
    _rbd->compute_moi();
    _rbd->center_of_mass += _rbd->linear_velocity * dt;
}

AdvanceRotationAndCOMWithCollisions::AdvanceRotationAndCOMWithCollisions(RB_sp rbd) :
    AdvanceRotationAndCOM(rbd) {
    if (!_rbd->has_vector_attribute("updated_positions")){
        _rbd->add_attribute<Vector>("updated_positions", DSAv());
    }
}

void AdvanceRotationAndCOMWithCollisions::solve(const double dt) {
    solve_no_collisions_and_populate_pos_and_updated_pos(dt);
    rbd_coll_handler.handle_collisions(_rbd, "updated_positions", dt);
}

void AdvanceRotationAndCOMWithCollisions::solve_no_collisions_and_populate_pos_and_updated_pos(const double dt) {
    // determine the current position of all the particles
    auto pos = _rbd->get_vector_attribute_span("positions");
    auto updated_pos = _rbd->get_vector_attribute_span("updated_positions");
    #pragma omp parallel for
    for (int i = 0; i < _rbd->n_particles(); i++){
        pos[i] = _rbd->get_vert_pos(i);
    }
    solve(dt);
    #pragma omp parallel for
    for (int i = 0; i < _rbd->n_particles(); i++){
        updated_pos[i] = _rbd->get_vert_pos(i);
    }

}


void AdvanceAngularVelocityAndVelocity::solve(const double dt){
    _torque.compute(_rbd, dt); // updates angular accel and center of mass acceleration
    _rbd->angular_momentum += _rbd->angular_accel * dt;
    _rbd->angular_velocity = _rbd->get_inverse_moi() * _rbd->angular_momentum;
    _rbd->linear_velocity += _rbd->com_accel * dt;
}







