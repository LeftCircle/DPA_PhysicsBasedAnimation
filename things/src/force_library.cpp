#include "force_library.h"


using namespace pba;



void SimpleGravityForce::compute(DynamicalStateData_sp dsd, const double dt) const {
	const size_t n = dsd->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		dsd->set_acceleration(i, dsd->get_acceleration(i) + _gravity);
	}
}