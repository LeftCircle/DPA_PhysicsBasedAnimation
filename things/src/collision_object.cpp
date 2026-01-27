#include "collision_object.h"

using namespace pba;


bool CollisionPlane::hit(
	const Vector& start_pos,
	const Vector& end_pos,
	const Vector& velocity,
	const double dt,
	CollisionHitInfo& hit_info) const {
    // Check wether the start and end points are on the same side of the plane
    // First check for the edge case where the point is on the plane
	double end_dot = (end_pos - _point_on_plane) * _plane_normal;
    if (std::abs(end_dot) < EPSILON) {
        hit_info.time_of_impact = NO_COLLISION;
        return false;
    }
	double start_dot = (start_pos - _point_on_plane) * _plane_normal;
	if (start_dot * end_dot > 0) {
		// The particle is on the same side of the collision plane
		hit_info.time_of_impact = NO_COLLISION;
		return false;
	} else{
		hit_info.time_of_impact = _plane_normal * ( _point_on_plane - start_pos ) / ( _plane_normal * velocity );
		if( hit_info.time_of_impact < 0.0 || hit_info.time_of_impact > dt ){
			hit_info.time_of_impact = NO_COLLISION;
			return false;
		}
		hit_info.position = start_pos + velocity * hit_info.time_of_impact;
		hit_info.normal = _plane_normal;
		return true;
	}
}



