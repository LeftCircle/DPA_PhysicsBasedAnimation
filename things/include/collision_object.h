#ifndef _COLLISION_OBJECT_H
#define _COLLISION_OBJECT_H

#include <memory>

#include "Vector.h"

namespace pba{

inline constexpr double EPSILON = std::numeric_limits<double>::epsilon();

inline constexpr double NO_COLLISION = std::numeric_limits<double>::infinity();
struct CollisionHitInfo {
	double time_of_impact;
	Vector position;
	Vector normal;
};

class CollisionObject
{
public:
	
	CollisionObject() = default;
	virtual ~CollisionObject() = default;

	// Packs the data into hit_info. If there is no collision, time_of_impact is set to NO_COLLISION
	virtual bool hit(
		const Vector& start_pos,
		const Vector& end_pos,
		const Vector& velocity,
		const double dt,
		CollisionHitInfo& hit_info
	) const = 0;
};

using CollisionObject_sp = std::shared_ptr<CollisionObject>;


class CollisionPlane : public CollisionObject
{
public:
	CollisionPlane(const Vector& point_on_plane, const Vector& plane_normal)
	: _point_on_plane(point_on_plane), _plane_normal(plane_normal.unitvector()) {}
	~CollisionPlane() = default;

	bool hit(const Vector& start_pos,
		const Vector& end_pos,
		const Vector& velocity,
		const double dt,
		CollisionHitInfo& hit_info
	) const override;


private:
	Vector _point_on_plane;
	Vector _plane_normal;
};

inline CollisionObject_sp create_collision_plane(const Vector& point_on_plane, const Vector& plane_normal){
	return std::make_shared<CollisionPlane>(point_on_plane, plane_normal);
}

} // end namespace pba

#endif