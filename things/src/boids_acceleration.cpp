#include "boids_acceleration.h"

using namespace pba;


Vector BoidBehaviors::get_acceleration_due_to_neighbors(
    	const Vector& pos,
		const Vector& vel,
		span<const Vector> neighbor_pos,
		span<const Vector> neighbor_vel,
        const BoidParams& params
	) const
{
    const size_t n_neighbors = neighbor_pos.size();
    std::vector<NeighborData> ndata(n_neighbors);
    for (size_t i = 0; i < n_neighbors; i++){
        ndata[i] = _make_neighbor_data(pos, vel, neighbor_pos[i], neighbor_vel[i], params);
    }
}


Vector BoidBehaviors::_get_neighbor_acceleration(const NeighborData& nb, const BoidParams& params){
    if (nb.weight < 1e-12) return Vector(0, 0, 0);

    double budget = params.threshold;
    Vector acc(0, 0, 0);

    acc = _collision_avoidance(nb.vec_to, params) * nb.weight;
    double acc_mag = acc.magnitude();
    if (acc_mag >= budget){
        return acc * budget / acc_mag; 
    }
    acc += _velocity_match(nb.vel_diff, params) * nb.weight;
    acc_mag = acc.magnitude();
    if (acc_mag >= budget){
        return acc * budget / acc_mag;
    }
    acc += _centering(nb.vec_to, params) * nb.weight;
    acc_mag = acc.magnitude();
    if (acc_mag >= budget){
        return acc * budget / acc_mag;
    }
    
}

NeighborData BoidBehaviors::_make_neighbor_data(
    const Vector& pos,
	const Vector& vel,
	const Vector& neighbor_pos,
	const Vector& neighbor_vel,
	const BoidParams& p
	) const
{
    Vector vec_to_n = neighbor_pos - pos;
    double range_limit = _range_limit(vec_to_n, p);
    double fov_limit = _fov_limiter(vec_to_n, p);
    return NeighborData{ vec_to_n, neighbor_vel - vel, range_limit * fov_limit, p};
};

double BoidBehaviors::_range_limit(const Vector& vec_to_point, const BoidParams& params) const{
    double d = vec_to_point.magnitude();
    if (d < params.range){
        return 1.0;
    } else if( d < (params.range + params.range_amp)){
        return 1.0 - (d - params.range) / params.range_amp;
    } else {
        return 0.0;
    }
}

double BoidBehaviors::_fov_limiter(
    const Vector& vec_to_point,
    const Vector& vel,
    const BoidParams& params) const
{
    double cos_ab = vec_to_point * vel / (vec_to_point.magnitude() * vel.magnitude());
    if (cos_ab >= params.cos_fov_angle){
        return 1.0;
    } else if (cos_ab > params.cos_fov_angle_plus_amp){
        return 1 - (params.cos_fov_angle - cos_ab) / (
            params.cos_fov_angle - params.cos_fov_angle_plus_amp
        );
    } else {
        return 0.0;
    }
}

Vector BoidBehaviors::_collision_avoidance(
    const Vector& a_to_b,
    const BoidParams& params) const
{
    double d = a_to_b.magnitude();
    if (d < 1e-10) return Vector(0, 0, 0);
    return -a_to_b * (params.ca_strength / (d * d));
}

Vector BoidBehaviors::_velocity_match(
		const Vector& vel_diff,
		const BoidParams& params
	) const
{
    return vel_diff * params.vm_strength;
}

Vector BoidBehaviors::_centering(const Vector& a_to_b, const BoidParams& params) const {
    return params.cent_strength * a_to_b;
}




