#include "particle_emitter.h"

using namespace pba;

void ParticleEmitter::emit(Vector& pos, Vector& vel) const noexcept {
    _generate_random_bounded_position(pos);
    _generate_random_bounded_velocity(vel);
}

void ParticleEmitter::_generate_random_bounded_position(Vector& pos) const noexcept {
    double rx = _bounds.lower_left().X() + (dist(get_rng())) * (_bounds.upper_right().X() - _bounds.lower_left().X());
    double ry = _bounds.lower_left().Y() + (dist(get_rng())) * (_bounds.upper_right().Y() - _bounds.lower_left().Y());
    double rz = _bounds.lower_left().Z() + (dist(get_rng())) * (_bounds.upper_right().Z() - _bounds.lower_left().Z());
    pos.set(rx, ry, rz);
}

void ParticleEmitter::_generate_random_bounded_velocity(Vector& vel) const noexcept {
    double speed = _min_speed + (dist(get_rng())) * (_max_speed - _min_speed);
    double theta = (dist(get_rng())) * 2.0 * M_PI;
    double phi = (dist(get_rng())) * M_PI;
    double vx = speed * sin(phi) * cos(theta);
    double vy = speed * sin(phi) * sin(theta);
    double vz = speed * cos(phi);
    vel.set(vx, vy, vz);
}
