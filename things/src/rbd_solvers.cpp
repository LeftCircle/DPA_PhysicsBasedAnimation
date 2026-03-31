#include "rbd_solvers.h"

using namespace pba;

void AdvanceRotationAndCOM::solve(const double dt) {
    Vector rotor = _rbd->angular_velocity * dt;
    _rbd->angular_rotation = rotation(rotor.unitvector(), -rotor.magnitude()) * _rbd->angular_rotation;
    _rbd->compute_moi();
    _rbd->center_of_mass += _rbd->linear_velocity * dt;
}








