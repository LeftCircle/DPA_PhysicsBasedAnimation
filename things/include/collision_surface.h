#ifndef _COLLISION_SURFACE_H
#define _COLLISION_SURFACE_H

#include <vector>
#include <memory>
#include <algorithm>

#include "collision_object.h"

namespace pba{

class CollisionSurface
{
public:
	CollisionSurface() = default;
	~CollisionSurface() = default;

	void add_collision_object(CollisionObject_sp obj){ _collision_objects.push_back( obj ); }
	
	void set_restitution(const double r){ restitution = std::clamp(r, 0.0, 1.0); }
	void set_sticky(const double s){ sticky = std::clamp(s, 0.0, 1.0); }

	const double get_restitution() const { return restitution; }
	const double get_sticky() const { return sticky; }

	bool hit(
		const Vector& start_pos,
		const Vector& end_pos,
		const Vector& velocity,
		const double dt,
		CollisionHitInfo& hit_info
	);

private:
	std::vector<CollisionObject_sp> _collision_objects;
	double restitution = 1.0;
	double sticky = 1.0;


};

using CollisionSurface_sp = std::shared_ptr<CollisionSurface>;

inline CollisionSurface_sp create_collision_surface(){ return std::make_shared<CollisionSurface>(); }

} // end namespace pba


#endif