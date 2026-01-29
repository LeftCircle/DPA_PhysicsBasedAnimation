#include "collision_object.h"

using namespace pba;

// Some internal helper functions for special cases
namespace {

void _handle_end_collision_on_plane(const Vector& end_pos, const Vector& plane_normal, const double dt, CollisionHitInfo& hit_info) {
	hit_info.time_of_impact = dt;
	hit_info.position = end_pos;
	hit_info.normal = plane_normal;
}

} // end anonymous namespace

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
		_handle_end_collision_on_plane(end_pos, _plane_normal, dt, hit_info);
		hit_info.time_of_impact = dt;
        hit_info.position = end_pos;
		hit_info.normal = _plane_normal;
		return true;
    }
	double start_dot = (start_pos - _point_on_plane) * _plane_normal;
	// Check to see if velocity is parallel to the plane or towards the plane
	double vel_dot = velocity * _plane_normal;
	if (std::abs(start_dot) <= EPSILON) {
		_handle_start_collision_on_plane(start_pos, _plane_normal, vel_dot);
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
	return _handle_general_case(start_pos, end_pos, velocity, dt, start_dot, end_dot, hit_info);
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



CollisionTriangle::CollisionTriangle(const Vector& v0, const Vector& v1, const Vector& v2)
	: _v0(v0), _v1(v1), _v2(v2) {
	_normal = (_v1 - _v0) ^ (_v2 - _v0);
	_normal.normalize();
}

bool CollisionTriangle::hit(
	const Vector& start_pos,
	const Vector& end_pos,
	const Vector& velocity,
	const double dt,
	CollisionHitInfo& hit_info) const {
	// First check for intersection with the plane of the triangle
	CollisionHitInfo plane_hit_info;
	CollisionPlane triangle_plane(_v0, _normal);
	bool hit = triangle_plane.hit(start_pos, end_pos, velocity, dt, plane_hit_info);
	if (!hit) {
		return false;
	}
	// Now check if the intersection point is within the triangle using barycentric coordinates
	Vector P = plane_hit_info.position;
	Vector v0 = _v1 - _v0;
	Vector v1 = _v2 - _v0;
	Vector v2 = P - _v0;

	double d00 = v0 * v0;
	double d01 = v0 * v1;
	double d11 = v1 * v1;
	double d20 = v2 * v0;
	double d21 = v2 * v1;

	double denom = d00 * d11 - d01 * d01;
	if (std::abs(denom) < EPSILON) {
		// Triangle is degenerate
		hit_info.time_of_impact = NO_COLLISION;
		return false;
	}
	double v = (d11 * d20 - d01 * d21) / denom;
	double w = (d00 * d21 - d01 * d20) / denom;
	double u = 1.0 - v - w;

	if (u >= 0.0 && v >= 0.0 && w >= 0.0) {
		// The intersection point is inside the triangle
		hit_info = plane_hit_info;
		return true;
	} else {
		hit_info.time_of_impact = NO_COLLISION;
		return false;
	}
}