#ifndef _COLLISION_SURFACE_H
#define _COLLISION_SURFACE_H

#include <vector>
#include <memory>

#include "collision_object.h"

namespace pba{

class CollisionSurface
{
public:
	CollisionSurface() = default;
	~CollisionSurface() = default;

	void add_collision_object(CollisionObject_sp obj){ _collision_objects.push_back( obj ); }

	

private:
	std::vector<CollisionObject_sp> _collision_objects;

};


} // end namespace pba


#endif