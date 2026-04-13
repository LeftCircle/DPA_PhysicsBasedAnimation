#include "boid_solvers.h"

using namespace pba;

void AdvanceBoidVelocityWithForces::solve(const double dt){
    _force_system->compute(_state_data, dt);
    const size_t n = _state_data->n_particles();
    auto pos = _state_data->get_vector_attribute_span("positions");
    auto vel = _state_data->get_vector_attribute_span("velocities");
    // for each boid, get its neighbors, then find acceleration
    for (size_t i = 0; i < n; i++){
        _occupancy_grid->accumulate_neighbor_cells(
            i, pos[i], Vector(0, 0, 0),
            [this, &pos, &vel](size_t index, idx_vec& n_idxs){
                return _boid_behaviors.get_acceleartion_due_to_neighbors()
            }
        );
    }

}