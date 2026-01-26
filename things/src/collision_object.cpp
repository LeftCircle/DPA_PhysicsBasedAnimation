#include "collision_object.h"

using namespace pba;


CollisionHitInfo CollisionPlane::hit(const Vector& start_pos, const Vector& end_pos, const Vector& velocity, const double dt) const {
    // Check wether the start and end points are on the same side of the plane
    // First check for the edge case where the point is on the plane
	double end_dot = (end_pos - _point_on_plane) * _plane_normal;
    if (std::abs(end_dot) < EPSILON) {
        return CollisionHitInfo{0.0, end_pos, _plane_normal};
    }
	double start_dot = (start_pos - _point_on_plane) * _plane_normal;
	if (start_dot * end_dot > 0) {
		return NO_COLLISION;
	} else{
		double t_hit;

	}

}



