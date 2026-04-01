#include "rbd_solvers.h"

using namespace pba;

void AdvanceRotationAndCOM::solve(const double dt) {
    Vector rotor = _rbd->angular_velocity * dt;
    _rbd->angular_rotation = rotation(rotor.unitvector(), -rotor.magnitude()) * _rbd->angular_rotation;
    _rbd->compute_moi();
    _rbd->center_of_mass += _rbd->linear_velocity * dt;
}

void AdvanceAngularVelocityAndVelocity::solve(const double dt){
    _torque.compute(_rbd, dt); // updates angular accel and center of mass acceleration
    _rbd->angular_momentum += _rbd->angular_accel * dt;
    _rbd->angular_velocity = _rbd->get_inverse_moi() * _rbd->angular_momentum;
    _rbd->linear_velocity += _rbd->com_accel * dt;
}







