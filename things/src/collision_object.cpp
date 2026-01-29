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
		hit_info.time_of_impact = dt;
        hit_info.position = end_pos;
		hit_info.normal = _plane_normal;
		return true;
    }
	double start_dot = (start_pos - _point_on_plane) * _plane_normal;
	// Check to see if velocity is parallel to the plane or towards the plane
	double vel_dot = velocity * _plane_normal;
	if (std::abs(start_dot) <= EPSILON) {
		// Special case where the particle is starting on the plane
		if (vel_dot > EPSILON) {
			// No collision. The particle starts on the plane but is already moving away. 
			hit_info.time_of_impact = NO_COLLISION;
			return false;
		} else {
			// We have a collision. The particle starts on the plane but is moving towards it, so the
			// time of impact is 0.
			hit_info.time_of_impact = 0.0;
			hit_info.position = start_pos;
			hit_info.normal = _plane_normal;
			return true;
		}
	}
	// Now we have the general collision case
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



