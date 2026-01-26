#ifndef _COLLISION_OBJECT_H
#define _COLLISION_OBJECT_H

#include <memory>

#include "Vector.h"

namespace pba{

inline constexpr double EPSILON = std::numeric_limits<double>::epsilon();

struct CollisionHitInfo {
	double time_of_impact;
	Vector contact_point;
	Vector contact_normal;
};

inline constexpr CollisionHitInfo NO_COLLISION = CollisionHitInfo(-1.0, Vector(), Vector());

class CollisionObject
{
public:
	
	CollisionObject() = default;
	virtual ~CollisionObject() = default;

	// Returns time of impact in [0,dt], or -1 if no collision
	virtual CollisionHitInfo hit(
		const Vector& start_pos,
		const Vector& end_pos,
		const Vector& velocity,
		const double dt
	) const = 0;
};

using CollisionObject_sp = std::shared_ptr<CollisionObject>;


class CollisionPlane : public CollisionObject
{
public:
	CollisionPlane(const Vector& point_on_plane, const Vector& plane_normal)
	: _point_on_plane(point_on_plane), _plane_normal(plane_normal.unitvector()) {}
	~CollisionPlane() = default;

	CollisionHitInfo hit(const Vector& start_pos,
		const Vector& end_pos,
		const Vector& velocity,
		const double dt
	) const override;


private:
	Vector _point_on_plane;
	Vector _plane_normal;
};

} // end namespace pba

#endif