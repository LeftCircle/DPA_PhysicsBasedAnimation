#include "force.h"


using namespace pba;


void ForceSystem::compute(DynamicalStateDataBase_sp dsd, const double dt) const {
	_reset_accelerations(dsd);
	for( const auto& force : _forces ){
		force->compute(dsd, dt);
	}
}

void ForceSystem::_reset_accelerations(DynamicalStateDataBase_sp dsd) const noexcept{
	const size_t n = dsd->n_particles();
	auto acc = dsd->get_vector_attribute_span("acceleration");
	std::fill(std::execution::par, acc.begin(), acc.end(), Vector(0, 0, 0));
}
