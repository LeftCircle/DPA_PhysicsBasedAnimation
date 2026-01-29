#include "collision_surface.h"


using namespace pba;


bool CollisionSurface::hit(
    const Vector& start_pos,
    const Vector& end_pos,
    const Vector& velocity,
    const double dt,
    CollisionHitInfo& hit_info
){
    CollisionHitInfo temp_hit;
    hit_info.time_of_impact = NO_COLLISION;
    for( const auto& obj : _collision_objects ){
        bool hit = obj->hit(start_pos, end_pos, velocity, dt, temp_hit);
        if (hit && temp_hit.time_of_impact < hit_info.time_of_impact) {
            hit_info = temp_hit;
        }
    }
    return hit_info.time_of_impact != NO_COLLISION;
}