#include "collision_handler.h"


using namespace pba;



void CollisionHandler::handle_collisions(DynamicalStateData& dsd, const std::string& updated_pos_attr_name, const double dt){
    const size_t n = dsd.n_particles();
    auto updated_pos_map_iter = dsd.get_vector_attribute(updated_pos_attr_name, 0); // just to check existence
    #pragma omp parallel for
    for( size_t i=0; i<n; i++ ){
        Vector start_pos = dsd.get_position(i);
        Vector updated_pos = dsd.get_vector_attribute(updated_pos_attr_name, i);
        Vector velocity = (updated_pos - start_pos) / static_cast<float>(dt);

        // Check collisions against all registered collision surfaces
        for (const auto& cs : collision_surfaces) {
            // For each collision object in the surface
        }
    }
}