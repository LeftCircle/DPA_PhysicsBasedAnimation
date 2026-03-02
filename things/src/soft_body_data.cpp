#include "soft_body_data.h"

using namespace pba;

void SoftBody::connect_all_particles_together() noexcept{
    std::span<const Vector> positions = get_vector_attribute_span("positions");
    for (size_t i = 0; i < n_particles(); i++){
        for (size_t j = i + 1; j < n_particles(); j++){
            edges.emplace_back(i, j, (positions[i] - positions[j]).magnitude());
        }        
    }
};



