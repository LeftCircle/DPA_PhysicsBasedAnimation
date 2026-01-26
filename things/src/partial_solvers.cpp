#include "partial_solvers.h"


using namespace pba;

void PartialSolverAdvancePosition::solve(const double dt){
    const size_t n = _state_data->n_particles();
    #pragma omp parallel for
    for( size_t i=0; i<n; i++ ){
        const Vector& pos = _state_data->get_position(i);
        const Vector& vel = _state_data->get_velocity(i);
        _state_data->set_position(i, pos + vel * static_cast<float>(dt) );
    }
}