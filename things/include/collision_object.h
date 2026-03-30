#ifndef _COLLISION_OBJECT_H
#define _COLLISION_OBJECT_H

#include <memory>
#include <cassert>
#include <iostream>
#include <limits>

#include "Vector.h"
#include "shapes.h"

namespace pba{

inline constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();
inline constexpr double MIN_END_DIST_FROM_COLLISION = 10 * EPSILON; 

//inline constexpr double NO_COLLISION = std::numeric_limits<double>::infinity();
//inline constexpr double NO_COLLISION_NEG = -std::numeric_limits<double>::infinity();

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

	// Packs the data into hit_info.
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
};

class CollisionTriangle : public CollisionObject
{

public:
	CollisionTriangle(const Triangle& triangle);
	CollisionTriangle(const Vector& v0, const Vector& v1, const Vector& v2);
	CollisionTriangle() = delete;
	~CollisionTriangle() = default;

	bool hit(const Vector& start_pos,
		const Vector& end_pos,
		const Vector& velocity,
		const double dt,
		CollisionHitInfo& hit_info
	) const override;

private:
	Triangle _triangle;
	Vector _edge1;
	Vector _edge2;
	Vector _normal;
	double _one_over_area_scale;
	double _det;

	bool _is_in_triangle(const Vector& p) const noexcept;
};

} // end namespace pba

#endif