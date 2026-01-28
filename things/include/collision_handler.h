#ifndef _COLLISION_HANDLER_H
#define _COLLISION_HANDLER_H


#include <vector>
#include <memory>
#include <string>

#include "collision_surface.h"
#include "dynamical_state_data.h"


namespace pba{

class CollisionHandler{

public:
    CollisionHandler() = default;
    ~CollisionHandler() = default;

    void register_collision_surface(const CollisionSurface_sp cs) { collision_surfaces.push_back(cs); }
    void handle_collisions(DynamicalStateData& dsd, const std::string& updated_pos_attr_name, const double dt);

private:
    std::vector<CollisionSurface_sp> collision_surfaces;

};

using CollisionHandler_sp =  std::shared_ptr<CollisionHandler> ;

inline CollisionHandler_sp create_collision_handler() { return std::make_shared<CollisionHandler>(); };

} // end namespace pba


#endif