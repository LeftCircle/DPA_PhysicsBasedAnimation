#ifndef _COLLISION_HANDLER_H
#define _COLLISION_HANDLER_H


#include <vector>
#include <memory>
#include <string>
#include <span>
#include <iostream>

#include "collision_surface.h"
#include "dynamical_state_data.h"


namespace pba{

struct CollisionHandleInfo {
    CollisionSurface_sp collision_surface;
    CollisionHitInfo hit_info;
};

class CollisionHandler{

public:
    CollisionHandler() = default;
    ~CollisionHandler() = default;

    void register_collision_surface(const CollisionSurface_sp cs) { collision_surfaces.push_back(cs); }
    void handle_collisions(DynamicalStateData_sp dsd, const std::string& updated_pos_attr_name, const double dt);

private:
    Vector _resolve_collision_against_static_object(
        const Vector& collision_position,
        const Vector& hit_normal,
        Vector& velocity,
        const double restitution, 
        const double sticky, 
        const double dt
    );

    std::vector<CollisionSurface_sp> collision_surfaces;

};

using CollisionHandler_sp =  std::shared_ptr<CollisionHandler> ;

inline CollisionHandler_sp create_collision_handler() { return std::make_shared<CollisionHandler>(); };

} // end namespace pba


#endif