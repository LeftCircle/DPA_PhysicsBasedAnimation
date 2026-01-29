#include "force.h"


using namespace pba;


void ForceSystem::compute(DynamicalStateData_sp dsd, const double dt) const {
	_reset_accelerations(dsd);
	for( const auto& force : _forces ){
		force->compute(dsd, dt);
	}
}

void ForceSystem::_reset_accelerations(DynamicalStateData_sp dsd) const noexcept{
	const size_t n = dsd->n_particles();
	#pragma omp parallel for
	for( size_t i=0; i<n; i++ ){
		dsd->set_acceleration(i, Vector(0.0, 0.0, 0.0));
	}
}
